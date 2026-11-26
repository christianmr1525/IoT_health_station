#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// IoT Health Monitoring Station v2.0 - Device Configuration Template
// Copy this file to `Firmware/config.h` and fill in your credentials.
// NEVER commit `Firmware/config.h` to version control.
// =============================================================================

// Unique device identifier (must match ^[A-Za-z0-9._-]{1,64}$)
#define DEVICE_ID "station-living-01"

// Primary and fallback WiFi credentials
#define WIFI_SSID_1 "your-wifi-ssid"
#define WIFI_PASS_1 "your-wifi-password"
#define WIFI_SSID_2 ""
#define WIFI_PASS_2 ""

// MQTT Broker configuration
#define MQTT_BROKER_HOST "192.168.1.100"
// Use 1883 for local dev or 8883 with USE_TLS = 1 in production
#define MQTT_BROKER_PORT 1883
#define MQTT_USERNAME "health_device"
#define MQTT_PASSWORD "change-me-device-password"

// Enable TLS (MQTTS on port 8883). Set CA_CERT_PEM when USE_TLS == 1.
#define USE_TLS 0
#if USE_TLS
static const char* MQTT_CA_CERT = R"EOF(
-----BEGIN CERTIFICATE-----
REPLACE_WITH_YOUR_BROKER_CA_CERTIFICATE
-----END CERTIFICATE-----
)EOF";
#endif

// Hardware pins (ESP32 DevKit V4)
#define PIN_MQ135 32
#define PIN_DHT11 23
#define PIN_LCD_RS 4
#define PIN_LCD_EN 5
#define PIN_LCD_D4 25
#define PIN_LCD_D5 27
#define PIN_LCD_D6 14
#define PIN_LCD_D7 13

// MQ-135 thermal stabilization period (ms). Real sensor needs 24-48h burn-in
// and at least 60s warm-up per cold boot before readings are marked stable.
#define MQ135_WARMUP_MS 60000UL

#endif // CONFIG_H
