/**
 * IoT Health & Environment Monitoring Station v2.0 (ESP32 DevKit V4)
 *
 * Key architectural fixes over v1.0:
 * 1. MAX30102 runs at matched 100 SPS without blocking 100 ms delays, preserving
 *    digital filter cutoff accuracy, with moving average + SQI validation.
 * 2. Thread-safe FreeRTOS design: sensor tasks push TelemetryFrame structs into
 *    a FreeRTOS queue; only loop() owns WiFi/MQTT sockets (zero race conditions).
 * 3. Offline-first 16x2 LCD updated directly from local spinlock-protected cache.
 * 4. Non-blocking WiFi/MQTT reconnect with exponential backoff (1s -> 30s) and
 *    graceful sensor degradation (no while(1) boot freezes).
 * 5. 32-frame RAM ring buffer (store-and-forward) during network outages.
 * 6. Structured JSON payloads + MQTT Last Will and Testament (LWT) status.
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#if USE_TLS
#include <WiFiClientSecure.h>
#endif
#include <MQUnifiedsensor.h>
#include <MQTT.h>
#include <DHT.h>
#include <MAX3010x.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "filters.h"
#include "config.h"

// ---------------------------- HARDWARE CONSTANTS ----------------------------
#define BOARD_NAME "ESP-32"
#define ADC_VOLTAGE_RES 3.3
#define MQ_TYPE "MQ-135"
#define ADC_BITS 12
#define RATIO_MQ135_CLEAN_AIR 3.6

// ---------------------------- NETWORKING & MQTT -----------------------------
WiFiMulti wifiMulti;
#if USE_TLS
WiFiClientSecure netClient;
#else
WiFiClient netClient;
#endif
MQTTClient mqttClient(512);

static char topicTelemetry[128];
static char topicStatus[128];

static unsigned long nextReconnectAttemptMs = 0;
static unsigned long reconnectBackoffMs = 1000;
static const unsigned long kMaxBackoffMs = 30000;
static uint32_t seqCounter = 0;

// ---------------------------- FREERTOS SYNC ---------------------------------
typedef struct {
  bool has_bpm;
  bool has_spo2;
  bool has_temp;
  bool has_hum;
  bool has_air;
  float bpm;
  float spo2_pct;
  float signal_quality;
  float temperature_c;
  float humidity_pct;
  float air_quality_ppm;
  bool max_ok;
  bool dht_ok;
  bool mq_ok;
  bool mq_warmup;
  bool finger_detected;
  unsigned long uptime_ms;
} TelemetryFrame;

static QueueHandle_t telemetryQueue = NULL;
static TaskHandle_t taskMaxHandle = NULL;
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
  bool max_ok;
  bool dht_ok;
  bool mq_ok;
  bool mq_warmup;
  bool finger_detected;
  bool has_bpm;
  bool has_spo2;
  bool has_temp;
  bool has_hum;
  bool has_air;
  float bpm;
  float spo2_pct;
  float signal_quality;
  float temperature_c;
  float humidity_pct;
  float air_quality_ppm;
} LocalState;

static LocalState sharedState = {
  false, false, false, true, false,
  false, false, false, false, false,
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// ---------------------------- RING BUFFER (OFFLINE) -------------------------
#define RING_CAP 32
#define JSON_BUF_SIZE 448

typedef struct {
  char payloads[RING_CAP][JSON_BUF_SIZE];
  size_t head;
  size_t tail;
  size_t count;
} TelemetryRingBuffer;

static TelemetryRingBuffer offlineRing = {{{0}}, 0, 0, 0};

static void ringPush(const char* json) {
  if (offlineRing.count == RING_CAP) {
    // Overwrite oldest frame when full
    offlineRing.head = (offlineRing.head + 1) % RING_CAP;
    offlineRing.count--;
  }
  strncpy(offlineRing.payloads[offlineRing.tail], json, JSON_BUF_SIZE - 1);
  offlineRing.payloads[offlineRing.tail][JSON_BUF_SIZE - 1] = '\0';
  offlineRing.tail = (offlineRing.tail + 1) % RING_CAP;
  offlineRing.count++;
}

static bool ringPop(char* out, size_t outSize) {
  if (offlineRing.count == 0) return false;
  strncpy(out, offlineRing.payloads[offlineRing.head], outSize - 1);
  out[outSize - 1] = '\0';
  offlineRing.head = (offlineRing.head + 1) % RING_CAP;
  offlineRing.count--;
  return true;
}

// ---------------------------- PERIPHERALS -----------------------------------
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
DHT dht(PIN_DHT11, DHT11);
MQUnifiedsensor MQ135(BOARD_NAME, ADC_VOLTAGE_RES, ADC_BITS, PIN_MQ135, MQ_TYPE);
MAX30105 maxSensor;

// ---------------------------- TIMERS ----------------------------------------
static unsigned long lastLcdSwitchMs = 0;
static unsigned long lastEnvReadMs = 0;
static unsigned long lastAirReadMs = 0;
static const unsigned long kLcdIntervalMs = 3000;
static const unsigned long kEnvIntervalMs = 15000;
static const unsigned long kAirIntervalMs = 10000;
static int lcdScreenIndex = 0;
static bool pulseReadPaused = false;

// ---------------------------- MAX30102 DSP ----------------------------------
// Matched sampling rate & digital filters (100 Hz)
static const auto kSamplingRate = maxSensor.SAMPLING_RATE_100SPS;
static const float kSamplingFrequency = 100.0f;
static const unsigned long kFingerThreshold = 10000;
static const unsigned int kFingerCooldownMs = 500;
static const float kEdgeThreshold = -2000.0f;
static const float kLowPassCutoff = 5.0f;
static const float kHighPassCutoff = 0.5f;
static const int kAveragingSamples = 5;
static const int kSampleThreshold = 3;

LowPassFilter lpRed(kLowPassCutoff, kSamplingFrequency);
LowPassFilter lpIr(kLowPassCutoff, kSamplingFrequency);
HighPassFilter hpRed(kHighPassCutoff, kSamplingFrequency);
Differentiator diffRed(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> avgBpm;
MovingAverageFilter<kAveragingSamples> avgSpo2;
MinMaxAvgStatistic statRed;
MinMaxAvgStatistic statIr;

static const float kSpO2_A = 1.5958422f;
static const float kSpO2_B = -34.6596622f;
static const float kSpO2_C = 112.6898759f;

static unsigned long lastHeartbeatMs = 0;
static unsigned long fingerTimestampMs = 0;
static bool fingerDetected = false;
static float lastDiff = NAN;
static bool crossed = false;
static unsigned long crossedTimeMs = 0;

// ---------------------------- SENSOR INIT -----------------------------------
static void initMQ135() {
  MQ135.setRegressionMethod(1); // PPM = a * ratio^b
  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  MQ135.init();

  float calcR0 = 0.0f;
  for (int i = 0; i < 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RATIO_MQ135_CLEAN_AIR);
    delay(20);
  }
  float avgR0 = calcR0 / 10.0f;
  bool ok = !isinf(avgR0) && !isnan(avgR0) && avgR0 > 0.0f;
  if (ok) {
    MQ135.setR0(avgR0);
  } else {
    Serial.println("[WARN] MQ-135 calibration failed; continuing in degraded mode.");
  }

  portENTER_CRITICAL(&stateMux);
  sharedState.mq_ok = ok;
  portEXIT_CRITICAL(&stateMux);
}

static void initMAX30102() {
  bool ok = maxSensor.begin() && maxSensor.setSamplingRate(kSamplingRate);
  if (!ok) {
    Serial.println("[WARN] MAX30102 not detected; continuing without pulse oximeter.");
  }
  portENTER_CRITICAL(&stateMux);
  sharedState.max_ok = ok;
  portEXIT_CRITICAL(&stateMux);
}

static void resetPulseFilters() {
  diffRed.reset();
  avgBpm.reset();
  avgSpo2.reset();
  lpRed.reset();
  lpIr.reset();
  hpRed.reset();
  statRed.reset();
  statIr.reset();
  lastDiff = NAN;
  crossed = false;
  lastHeartbeatMs = 0;
}

// ---------------------------- TELEMETRY ENCODING ----------------------------
static void enqueueCurrentFrame(bool hasBpm, bool hasSpo2, bool hasTemp, bool hasHum, bool hasAir) {
  if (telemetryQueue == NULL) return;

  TelemetryFrame frame;
  portENTER_CRITICAL(&stateMux);
  frame.has_bpm = hasBpm && sharedState.has_bpm;
  frame.has_spo2 = hasSpo2 && sharedState.has_spo2;
  frame.has_temp = hasTemp && sharedState.has_temp;
  frame.has_hum = hasHum && sharedState.has_hum;
  frame.has_air = hasAir && sharedState.has_air;
  frame.bpm = sharedState.bpm;
  frame.spo2_pct = sharedState.spo2_pct;
  frame.signal_quality = sharedState.signal_quality;
  frame.temperature_c = sharedState.temperature_c;
  frame.humidity_pct = sharedState.humidity_pct;
  frame.air_quality_ppm = sharedState.air_quality_ppm;
  frame.max_ok = sharedState.max_ok;
  frame.dht_ok = sharedState.dht_ok;
  frame.mq_ok = sharedState.mq_ok;
  frame.mq_warmup = sharedState.mq_warmup;
  frame.finger_detected = sharedState.finger_detected;
  portEXIT_CRITICAL(&stateMux);
  frame.uptime_ms = millis();

  xQueueSend(telemetryQueue, &frame, 0);
}

static void formatTelemetryJson(const TelemetryFrame& f, char* out, size_t outSize) {
  seqCounter++;
  char metricsBuf[256] = "{";
  bool first = true;

  auto appendFloat = [&](const char* key, float val, const char* fmt) {
    if (!isfinite(val)) return;
    char kv[64];
    snprintf(kv, sizeof(kv), "%s\"%s\":", first ? "" : ",", key);
    strncat(metricsBuf, kv, sizeof(metricsBuf) - strlen(metricsBuf) - 1);
    snprintf(kv, sizeof(kv), fmt, val);
    strncat(metricsBuf, kv, sizeof(metricsBuf) - strlen(metricsBuf) - 1);
    first = false;
  };

  if (f.has_bpm) appendFloat("bpm", f.bpm, "%.1f");
  if (f.has_spo2) appendFloat("spo2_pct", f.spo2_pct, "%.1f");
  if (f.has_bpm || f.has_spo2) appendFloat("signal_quality", f.signal_quality, "%.2f");
  if (f.has_temp) appendFloat("temperature_c", f.temperature_c, "%.1f");
  if (f.has_hum) appendFloat("humidity_pct", f.humidity_pct, "%.1f");
  if (f.has_air) appendFloat("air_quality_ppm", f.air_quality_ppm, "%.2f");
  strncat(metricsBuf, "}", sizeof(metricsBuf) - strlen(metricsBuf) - 1);

  snprintf(
    out,
    outSize,
    "{\"device_id\":\"%s\",\"seq\":%lu,\"uptime_ms\":%lu,"
    "\"metrics\":%s,"
    "\"sensor_status\":{\"max30102\":%s,\"dht11\":%s,\"mq135\":%s,"
    "\"mq135_warmup\":%s,\"finger_detected\":%s}}",
    DEVICE_ID,
    (unsigned long)seqCounter,
    (unsigned long)f.uptime_ms,
    metricsBuf,
    f.max_ok ? "true" : "false",
    f.dht_ok ? "true" : "false",
    f.mq_ok ? "true" : "false",
    f.mq_warmup ? "true" : "false",
    f.finger_detected ? "true" : "false"
  );
}

// ---------------------------- NON-BLOCKING NETWORK --------------------------
static void maintainNetworkNonBlocking() {
  if (mqttClient.connected()) {
    mqttClient.loop();
    reconnectBackoffMs = 1000;
    return;
  }

  unsigned long now = millis();
  if (now < nextReconnectAttemptMs) {
    return;
  }

  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("[NET] WiFi disconnected; scheduling retry...");
    nextReconnectAttemptMs = now + reconnectBackoffMs;
    reconnectBackoffMs = min(reconnectBackoffMs * 2, kMaxBackoffMs);
    return;
  }

  Serial.print("[NET] Connecting MQTT broker ");
  Serial.println(MQTT_BROKER_HOST);
  bool connected = false;
  if (strlen(MQTT_USERNAME) > 0) {
    connected = mqttClient.connect(DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD);
  } else {
    connected = mqttClient.connect(DEVICE_ID);
  }

  if (connected) {
    Serial.println("[NET] MQTT connected.");
    char statusPayload[128];
    snprintf(statusPayload, sizeof(statusPayload), "{\"device_id\":\"%s\",\"status\":\"online\"}", DEVICE_ID);
    mqttClient.publish(topicStatus, statusPayload, true, 1);
    reconnectBackoffMs = 1000;
    nextReconnectAttemptMs = 0;

    // Flush store-and-forward ring buffer
    char buffered[JSON_BUF_SIZE];
    while (mqttClient.connected() && ringPop(buffered, sizeof(buffered))) {
      mqttClient.publish(topicTelemetry, buffered, false, 1);
      mqttClient.loop();
    }
  } else {
    Serial.println("[NET] MQTT connect failed; backing off.");
    nextReconnectAttemptMs = now + reconnectBackoffMs;
    reconnectBackoffMs = min(reconnectBackoffMs * 2, kMaxBackoffMs);
  }
}

static void publishOrBuffer(const TelemetryFrame& frame) {
  char json[JSON_BUF_SIZE];
  formatTelemetryJson(frame, json, sizeof(json));

  if (mqttClient.connected()) {
    if (!mqttClient.publish(topicTelemetry, json, false, 1)) {
      ringPush(json);
    }
  } else {
    ringPush(json);
  }
}

// ---------------------------- SENSOR TASKS ----------------------------------
static void processMAX30102Sample() {
  auto sample = maxSensor.readSample(25);
  if (!sample.valid) {
    return;
  }

  unsigned long now = millis();
  if (sample.red > kFingerThreshold) {
    if (now - fingerTimestampMs > kFingerCooldownMs) {
      fingerDetected = true;
    }
  } else {
    resetPulseFilters();
    fingerDetected = false;
    fingerTimestampMs = now;
    portENTER_CRITICAL(&stateMux);
    sharedState.finger_detected = false;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  portENTER_CRITICAL(&stateMux);
  sharedState.finger_detected = true;
  portEXIT_CRITICAL(&stateMux);

  float redFiltered = lpRed.process(sample.red);
  float irFiltered = lpIr.process(sample.ir);
  statRed.process(redFiltered);
  statIr.process(irFiltered);

  float acRed = hpRed.process(redFiltered);
  float currentDiff = diffRed.process(acRed);

  if (!isnan(currentDiff) && !isnan(lastDiff)) {
    if (lastDiff > 0.0f && currentDiff < 0.0f) {
      crossed = true;
      crossedTimeMs = now;
    }
    if (currentDiff > 0.0f) {
      crossed = false;
    }

    if (crossed && currentDiff < kEdgeThreshold) {
      if (lastHeartbeatMs != 0 && (crossedTimeMs - lastHeartbeatMs) > 300) {
        float instantBpm = 60000.0f / (float)(crossedTimeMs - lastHeartbeatMs);
        float redAvg = statRed.average();
        float irAvg = statIr.average();
        float rRed = (redAvg > 0.0f) ? (statRed.maximum() - statRed.minimum()) / redAvg : 0.0f;
        float rIr = (irAvg > 0.0f) ? (statIr.maximum() - statIr.minimum()) / irAvg : 0.0f;
        float rVal = (rIr > 0.0f) ? (rRed / rIr) : 0.0f;
        float instantSpo2 = kSpO2_A * rVal * rVal + kSpO2_B * rVal + kSpO2_C;

        if (instantBpm >= 35.0f && instantBpm <= 220.0f && instantSpo2 >= 70.0f && instantSpo2 <= 100.0f) {
          float smoothedBpm = avgBpm.process(instantBpm);
          float smoothedSpo2 = avgSpo2.process(instantSpo2);
          float sqi = min(1.0f, (float)avgBpm.count() / (float)kAveragingSamples);

          if (avgBpm.count() >= kSampleThreshold) {
            portENTER_CRITICAL(&stateMux);
            sharedState.has_bpm = true;
            sharedState.has_spo2 = true;
            sharedState.bpm = smoothedBpm;
            sharedState.spo2_pct = min(100.0f, max(70.0f, smoothedSpo2));
            sharedState.signal_quality = sqi;
            portEXIT_CRITICAL(&stateMux);
            enqueueCurrentFrame(true, true, false, false, false);
          }
        }
        statRed.reset();
        statIr.reset();
      }
      crossed = false;
      lastHeartbeatMs = crossedTimeMs;
    }
  }
  lastDiff = currentDiff;
}

void TaskMAX30102(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    bool maxOk = false;
    portENTER_CRITICAL(&stateMux);
    maxOk = sharedState.max_ok;
    portEXIT_CRITICAL(&stateMux);

    if (maxOk) {
      processMAX30102Sample();
      vTaskDelay(pdMS_TO_TICKS(5)); // Keeps 100 SPS FIFO drained accurately
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

static void readDHT11() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  bool ok = !isnan(t) && !isnan(h) && t > -20.0f && t < 70.0f && h >= 0.0f && h <= 100.0f;

  portENTER_CRITICAL(&stateMux);
  sharedState.dht_ok = ok;
  if (ok) {
    sharedState.temperature_c = t;
    sharedState.humidity_pct = h;
    sharedState.has_temp = true;
    sharedState.has_hum = true;
  }
  portEXIT_CRITICAL(&stateMux);

  if (ok) {
    enqueueCurrentFrame(false, false, true, true, false);
  } else {
    Serial.println("[WARN] DHT11 read failed.");
  }
}

static void readMQ135() {
  bool warmup = millis() < MQ135_WARMUP_MS;
  bool mqOk = false;
  portENTER_CRITICAL(&stateMux);
  mqOk = sharedState.mq_ok;
  sharedState.mq_warmup = warmup;
  portEXIT_CRITICAL(&stateMux);

  if (!mqOk) return;

  MQ135.update();
  float ppm = MQ135.readSensor();
  bool valid = isfinite(ppm) && ppm >= 0.0f && ppm <= 5000.0f;

  portENTER_CRITICAL(&stateMux);
  if (valid && !warmup) {
    sharedState.air_quality_ppm = ppm;
    sharedState.has_air = true;
  }
  portEXIT_CRITICAL(&stateMux);

  if (valid && !warmup) {
    enqueueCurrentFrame(false, false, false, false, true);
  }
}

// ---------------------------- OFFLINE-FIRST LCD -----------------------------
static void displayLocalScreen() {
  LocalState snap;
  portENTER_CRITICAL(&stateMux);
  snap = sharedState;
  portEXIT_CRITICAL(&stateMux);

  const char* netBadge = mqttClient.connected() ? "OK" : "OFF";
  lcd.clear();
  lcd.setCursor(0, 0);

  if (lcdScreenIndex == 0) {
    lcd.printf("Temp       [%3s]", netBadge);
    lcd.setCursor(0, 1);
    if (snap.has_temp) lcd.printf("%.1f C", snap.temperature_c);
    else lcd.print("Sensor N/A");
  } else if (lcdScreenIndex == 1) {
    lcd.printf("Humidity   [%3s]", netBadge);
    lcd.setCursor(0, 1);
    if (snap.has_hum) lcd.printf("%.1f %% RH", snap.humidity_pct);
    else lcd.print("Sensor N/A");
  } else if (lcdScreenIndex == 2) {
    lcd.printf("Air (CO)   [%3s]", netBadge);
    lcd.setCursor(0, 1);
    if (snap.mq_warmup) lcd.print("Warming up...");
    else if (snap.has_air) lcd.printf("%.2f ppm", snap.air_quality_ppm);
    else lcd.print("Sensor N/A");
  } else if (lcdScreenIndex == 3) {
    lcd.printf("Heart Rate [%3s]", netBadge);
    lcd.setCursor(0, 1);
    if (!snap.max_ok) lcd.print("Sensor N/A");
    else if (!snap.finger_detected) lcd.print("Place finger");
    else if (snap.has_bpm) lcd.printf("%.0f bpm (Q:%.0f%%)", snap.bpm, snap.signal_quality * 100.0f);
    else lcd.print("Measuring...");
  } else {
    lcd.printf("SpO2       [%3s]", netBadge);
    lcd.setCursor(0, 1);
    if (!snap.max_ok) lcd.print("Sensor N/A");
    else if (!snap.finger_detected) lcd.print("Place finger");
    else if (snap.has_spo2) lcd.printf("%.1f %%", snap.spo2_pct);
    else lcd.print("Measuring...");
  }

  lcdScreenIndex = (lcdScreenIndex + 1) % 5;
}

// ---------------------------- SETUP & LOOP ----------------------------------
void setup() {
  Serial.begin(115200);
  snprintf(topicTelemetry, sizeof(topicTelemetry), "health/devices/%s/telemetry", DEVICE_ID);
  snprintf(topicStatus, sizeof(topicStatus), "health/devices/%s/status", DEVICE_ID);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Health Station");
  lcd.setCursor(0, 1);
  lcd.print("v2.0 Starting...");

  telemetryQueue = xQueueCreate(16, sizeof(TelemetryFrame));

  dht.begin();
  initMQ135();
  initMAX30102();

  WiFi.mode(WIFI_STA);
  if (strlen(WIFI_SSID_1) > 0) wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);
  if (strlen(WIFI_SSID_2) > 0) wifiMulti.addAP(WIFI_SSID_2, WIFI_PASS_2);

#if USE_TLS
  netClient.setCACert(MQTT_CA_CERT);
#endif
  mqttClient.begin(MQTT_BROKER_HOST, MQTT_BROKER_PORT, netClient);

  static char lwtPayload[128];
  snprintf(lwtPayload, sizeof(lwtPayload), "{\"device_id\":\"%s\",\"status\":\"offline\"}", DEVICE_ID);
  mqttClient.setWill(topicStatus, lwtPayload, true, 1);

  xTaskCreatePinnedToCore(
    TaskMAX30102,
    "MAX30102Task",
    4096,
    NULL,
    2,
    &taskMaxHandle,
    1
  );
}

void loop() {
  if (Serial.available()) {
    char cmd = (char)Serial.read();
    if (cmd == 'p' && !pulseReadPaused && taskMaxHandle != NULL) {
      vTaskSuspend(taskMaxHandle);
      pulseReadPaused = true;
      Serial.println("[CTRL] MAX30102 paused.");
    } else if (cmd == 'r' && pulseReadPaused && taskMaxHandle != NULL) {
      vTaskResume(taskMaxHandle);
      pulseReadPaused = false;
      Serial.println("[CTRL] MAX30102 resumed.");
    }
  }

  maintainNetworkNonBlocking();

  unsigned long now = millis();
  if (now - lastEnvReadMs >= kEnvIntervalMs) {
    lastEnvReadMs = now;
    readDHT11();
  }
  if (now - lastAirReadMs >= kAirIntervalMs) {
    lastAirReadMs = now;
    readMQ135();
  }

  // Drain FreeRTOS queue from single socket-owning thread
  TelemetryFrame frame;
  while (telemetryQueue != NULL && xQueueReceive(telemetryQueue, &frame, 0) == pdTRUE) {
    publishOrBuffer(frame);
  }

  if (now - lastLcdSwitchMs >= kLcdIntervalMs) {
    lastLcdSwitchMs = now;
    displayLocalScreen();
  }
}
