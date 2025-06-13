//////////////////////////// LIBRERIAS //////////////////////////////////////////////////////////

#include <WiFi.h>               // Librería para conexión WiFi
#include <WiFiMulti.h>          // Librería para gestionar múltiples redes WiFi
#include <MQUnifiedsensor.h>    // Librería para sensor MQ-135
#include <MQTT.h>               // Librería para comunicación MQTT
#include <DHT.h>                // Librería para sensor DHT
#include <MAX3010x.h>           // Librería para sensor MAX3010x (frecuencia cardíaca y SpO2)
#include <LiquidCrystal.h>      // Librería para controlar LCD
#include "filters.h"            // Archivo con filtros para el procesamiento de señales
#include "data.h"               // Archivo para almacenamiento o gestión de datos (wifi contrasenas y usuarios, brokerMQTT)

//////////////////////////// DATOS DEFINE //////////////////////////////////////////////////////////

#define placa "ESP-32"                  // Nombre de la placa utilizada
#define Voltage_Resolution 3.3          // Resolución de voltaje del ADC (en volts)
#define type "MQ-135"                   // Tipo de sensor MQ utilizado
#define ADC_Bit_Resolution 12           // Resolución del ADC del ESP32
#define RatioMQ135CleanAir 3.6          // Relación estándar en aire limpio para calibrar el MQ-135

//////////////////////////// OBJETOS Wifi y MQTT //////////////////////////////////////////////////////////

WiFiMulti wifiMulti;         // Objeto para gestionar múltiples redes WiFi
WiFiClient net;              // Cliente de red para conexión MQTT
MQTTClient clienteMQTT;      // Cliente MQTT para publicar y suscribirse a mensajes

//////////////////////////// OBJETOS FreeRTOS //////////////////////////////////////////////////////////

TaskHandle_t TaskMAX30105Handle = NULL;
bool lecturaActiva = true;

//////////////////////////// CONEXION PINES IO //////////////////////////////////////////////////////////

#define MQ_pin 32                             // Pin analógico al que está conectado el sensor MQ135
LiquidCrystal lcd(4, 5, 25, 27, 14, 13);      // Configuración del LCD: RS = 4, E = 5, D4 = 25, D5 = 27, D6 = 14, D7 = 13
int DHT_pin = 23;                             // Pin digital al que está conectado el sensor DHT11

//////////////////////////// MQTT Topicos //////////////////////////////////////////////////////////////

String publish_topic = "/Clase";              // Topico en donde se publica mensaje
String suscribe_topic = "DB_last_values";     // Topico al cual se suscribe el ESP32

//////////////////////////// VARIABLES LCD /////////////////////////////////////////////////////////////

// Ultimos valores a mostrar en LCD
String lcd_temperature = ""; // Último valor de temperatura recibido para mostrar en el LCD
String lcd_humidity = "";    // Último valor de humedad recibido para mostrar en el LCD
String lcd_gas = "";         // Último valor de concentración de gas (CO) recibido para mostrar en el LCD
String lcd_bpm = "";         // Último valor de frecuencia cardíaca (bpm) recibido para mostrar en el LCD
String lcd_spo2 = "";        // Último valor de saturación de oxígeno (SpO2) recibido para mostrar en el LCD

// Control de transicion de mensajes 
unsigned long lastUpdateTime = 0;           // Marca de tiempo de la última actualización de pantalla
const unsigned long updateInterval = 3000;  // Intervalo de tiempo para cambiar entre pantallas (en millisegundos)
int screenIndex = 0;                        // Índice para determinar qué pantalla se muestra actualmente

//////////////////////////// TIEMPOS DE MUESTREO DE SENSORES ///////////////////////////////////////////

unsigned long lastTempHumRead = 0;    // Tiempo de la última lectura de temperatura y humedad
unsigned long lastCORead = 0;         // Tiempo de la última lectura del sensor de CO

const unsigned long tempHumInterval = 20 * 1000;  // Intervalo entre lecturas de temperatura y humedad (en milisegundos)
const unsigned long coInterval     = 10 * 1000;   // Intervalo entre lecturas del sensor de CO (en milisegundos)

bool newTempHumData = false;    // Bandera para indicar que hay nuevos datos de temperatura y humedad disponibles
bool newCO2Data = false;        // Bandera para indicar que hay nuevos datos del sensor de CO disponibles

//////////////////////////// VARIABLES MQ-135 //////////////////////////////////////////////////////////

// Objeto para el sensor de calidad del aire MQ135 
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ_pin, type);  // (placa, resolución de voltaje, resolución del ADC, pin de conexión, tipo de gas)

float gas;    // Variable para almacenar la concentración del gas detectado (en este caso, CO en ppm)

//////////////////////////// VARIABLES DHT11 //////////////////////////////////////////////////////////

DHT dht(DHT_pin, DHT11);  // Objeto para el sensor DHT11 conectado al pin especificado
float temperature;        // Variable para almacenar la temperatura (Celsius)
float humidity;           // Variable para almacenar la humedad (%)

//////////////////////////// VARIABLES MAX30105 ///////////////////////////////////////////////////////

MAX30105 sensor;                // Objeto para controlar el sensor MAX30105   
bool newMAX30105Data = false;   // Bandera que indica si hay nuevos datos procesados disponibles

// Configuración de muestreo
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;   // Tasa de muestreo: 400 muestras por segundo
const float kSamplingFrequency = 400.0;                   // Frecuencia de muestreo en Hz (para filtros)

// Detección de dedo
const unsigned long kFingerThreshold = 10000;           // Umbral mínimo de luz reflejada para considerar que hay un dedo
const unsigned int kFingerCooldownMs = 500;             // Tiempo de espera en ms para confirmar/remover la presencia del dedo

// Umbral para detección de latido (ajustar para sensores como el MAX30100)
const float kEdgeThreshold = -2000.0;   // Umbral de caída para detectar el pulso cardíaco

// Parámetros de los filtros
const float kLowPassCutoff = 5.0;     // Frecuencia de corte para el filtro pasa bajas (Hz)
const float kHighPassCutoff = 0.5;    // Frecuencia de corte para el filtro pasa altas (Hz)

// Promedio móvil
const bool kEnableAveraging = false;    // Habilita o deshabilita el uso de promedios
const int kAveragingSamples = 5;        // Número de muestras para el promedio móvil
const int kSampleThreshold = 5;         // Muestras mínimas necesarias antes de mostrar el promedio

// Instancias de filtros
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);    // Filtro pasa bajas para señal roja
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);     // Filtro pasa bajas para señal IR 
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);     // Filtro pasa altas para eliminar DC
Differentiator differentiator(kSamplingFrequency);        // Derivada para detectar cruce por cero
MovingAverageFilter<kAveragingSamples> averager_bpm;      // Filtro promedio móvil para BPM
MovingAverageFilter<kAveragingSamples> averager_r;        // Filtro promedio móvil para valor R
MovingAverageFilter<kAveragingSamples> averager_spo2;     // Filtro promedio móvil para SpO2

// Estadisticas para el calculo de oximetria
MinMaxAvgStatistic stat_red;    // Estadísticas para señal roja
MinMaxAvgStatistic stat_ir;     // Estadísticas para señal IR

// Coeficientes de calibración para calcular SpO2 a partir del valor R
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp del último latido detectado
long last_heartbeat = 0;

// Timestamp para la última detección de dedo
long finger_timestamp = 0;
bool finger_detected = false;

// Último valor derivado para detectar cruce por cero
float last_diff = NAN;
bool crossed = false;   // Indica si se cruzó el cero recientemente
long crossed_time = 0;  // Tiempo en que ocurrió el cruce

float bpm;    // Frecuencia cardíaca 
float r;      // Valor R (relación de amplitudes entre señales roja e infrarroja)
float spo2;   // Nivel de oxígeno en sangre (SpO2)

// Variables promediadas
float average_bpm;
float average_r;
float average_spo2;

/////////////////// FUNCIONES MQTT y WiFi //////////////////////////////////////////////////////////////

// Funcion para conexion al WiFi y al MQTT
void conectar(){  
  Serial.print("Conectando con WiFi...");
  lcd.clear();
  lcd.print("Conectando a");
  lcd.setCursor(0, 1);
  lcd.print("WiFi......");
  lcd.setCursor(0, 0);
  delay(1000);

  // Espera hasta que se conecte a una de las redes WiFi disponibles en wifiMulti
  while(wifiMulti.run() != WL_CONNECTED){
    Serial.print(".");
    delay(1000);
  }

  // Conexion WiFi exitosa 
  Serial.print("Se conecto a WiFi: ");
  Serial.print(WiFi.SSID());
  Serial.print(" -  ");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.print("Conectado a");
  lcd.setCursor(0, 1);
  lcd.print("WiFi");
  lcd.setCursor(0, 0);
  delay(1000);

  // Conexion al broker MQTT
  Serial.print("Conectando a MQTT***");
  lcd.clear();
  lcd.print("Conectando con");
  lcd.setCursor(0, 1);
  lcd.print("MQTT......");
  lcd.setCursor(0, 0);
  delay(1000);
  

  // Intenta conectar con el broker MQTT hasta que tenga éxito
  while(!clienteMQTT.connect(NombreESP)){
    Serial.print("*");
    delay(1000);
  }

  // Conexion MQTT exitosa
  Serial.println("\nConectado a MQTT");
  lcd.clear();
  lcd.print("Conectado a");
  lcd.setCursor(0,1);
  lcd.print("MQTT");
  lcd.setCursor(0, 0);
  delay(1000);
  
  // Suscripcion al topic MQTT
  clienteMQTT.subscribe("/saludo");
}

/////////////////// FUNCIONES MQ-135 ///////////////////////////////////////////////////////////////////

void MQ135_setup(){ // Funcion que inicializa el sensor MQ135

  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen   | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton   | 34.668 | -3.369
  */

  MQ135.setRegressionMethod(1); // PPM = a*ratio^b
  MQ135.setA(605.18);
  MQ135.setB(-3.937);

  MQ135.init();   // Inicializa el senso

  // Calibración del sensor MQ135 
  Serial.print("Calibrando, por favor espere");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); 
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!."); 

  // Verificación de errores de calibración 
  if(isinf(calcR0)) {Serial.println("Precaucion: Problema en conexion, R0 es infinito (Circuito abierto detectado), por favor verifique el cableado y fuente de voltaje"); while(1);}
  if(calcR0 == 0){Serial.println("Precaucion: Problema en conexion encontrado, R0 es cero (Pin analogico en cortocircuito a tierra) por favor verifique el cableado y fuente de voltaje"); while(1);}
  MQ135.serialDebug(true);
}

void MQ135_read(){ // Funcion de lectura de sensor MQ135
  MQ135.update();  
  float g = MQ135.readSensor();  
  if (g > 0) {
    gas = g; 
    newCO2Data = true;
  } else {
    Serial.println("MQ135: Lectura inválida o cero");
    newCO2Data = false;
  }
}

/////////////////// FUNCIONES DHT11 ///////////////////////////////////////////////////////////////////

void DHT11_read(){ // Funcion de lectura DHT11
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h)) {
    temperature = t;
    humidity = h;
    newTempHumData = true;
  } else {
    Serial.println("DHT11: Lectura inválida");
    newTempHumData = false;
  }
}

/////////////////// FUNCIONES MAX30105 ///////////////////////////////////////////////////////////////////

void MAX30105_setup(){  // Funcion que inicializa el sensor MAX30105
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor inicializado");
  }
  else {
    Serial.println("Sensor no encontrado");  
    while(1);
  }  
}

void MAX30105_read(){  // Funcion de lectura MAX30105
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

  // Detecta si el dedo esta presente mediante el valor de la luz IR
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Se reinician los filtros y estadisticas si se retira el dedo
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Calculo de estadisticas de la oximetria
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Detecta pulso usando valores de la señal IR
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Verifica que las diferencias sean validad (que no haya NaN)
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Verifica si el pulso pasa por el cruce cero
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      // Reinicia la bandera si la señal sube nuevamente
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detecta que el pulse pase por el umbral de flanco descendente
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Mostrar resultados de BMP y SpO2
          bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          r = rred/rir;
          spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          
          if(bpm > 50 && bpm < 250) {
            // Si el promedio esta activado
            if(kEnableAveraging) {
              average_bpm = averager_bpm.process(bpm);
              average_r = averager_r.process(r);
              average_spo2 = averager_spo2.process(spo2);
  
              // Se muestran los resultados solo si hay suficientes muestras para promediar
              if(averager_bpm.count() >= kSampleThreshold) {
                newMAX30105Data = true;
                Serial.print("Heart Rate (average, bpm): ");
                Serial.println(average_bpm);  
                Serial.print("SpO2 (average, %): ");
                Serial.println(average_spo2);   
              }
            }
            else {
              // Se muestran los valores instantáneos si no se promedia
              newMAX30105Data = true;
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm); 
              Serial.print("SpO2 (current, %): ");
              Serial.println(spo2);
            }
          }

          // Reiniciar estadisticas despues de detectar un pulso
          stat_red.reset();
          stat_ir.reset();
        }
    
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  }
}

///////////////////////////////// MQTT publish ////////////////////////////////////////////////////////

void MQTT_publish_temperature_humidity() {  // Funcion que publica en temperatura y humedad en topic MQTT
  if (!clienteMQTT.connected()) {
    conectar();
  }
  if (newTempHumData) {
    clienteMQTT.publish(publish_topic, "temperature," + String(temperature));
    clienteMQTT.publish(publish_topic, "humidity," + String(humidity));
    newTempHumData = false; 
  }
}

void MQTT_publish_co() {        // Funcion que publica concentracion de CO en topic MQTT
  if (!clienteMQTT.connected()) {
    conectar();
  }
  if (newCO2Data) {
    clienteMQTT.publish(publish_topic, "CO concentration," + String(gas));
    newCO2Data = false; 
  }
}

void MQTT_publish_max30105() {      // Funcion que publica concentracion de CO en topic MQTT
  if (!clienteMQTT.connected()) {
    conectar();
  }
  if (finger_detected) {
    clienteMQTT.publish(publish_topic, "bpm," + String(bpm));
    clienteMQTT.publish(publish_topic, "SpO2," + String(spo2));
  }
}

///////////////////////////////// MQTT suscribe and read + LCD DISPLAY ////////////////////////////////////////////////////////


void callback(String &topic, String &payload) { // Funcion que recibe mensaje desde base de datos por MQTT y los manipula para monstrarlos en el LCD
  Serial.print("\n📥 Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payload);

  int start = 0;
  while (true) {
    int end = payload.indexOf('\n', start);         // Busca el fin de una línea en el mensaje
    String line = payload.substring(start, end);    // Extrae una línea completa del mensaje
    Serial.print("➡️  ");
    Serial.println(line);

    int sep = line.indexOf(',');                    // Busca la coma que separa el tipo de dato y el valor
    if (sep != -1) {
      String data_type = line.substring(0, sep);    // Extrae el tipo de dato (temperatura, humedad, etc)
      String value = line.substring(sep + 1);       // Extrae el valor numérico

      // Asigna el valor recibido a la variable correspondiente para mostrarlo en el LCD, pero aqui no lo muestra aun
      if (data_type == "temperature") lcd_temperature = value + " C";
      else if (data_type == "humidity") lcd_humidity = value + " %";
      else if (data_type == "CO concentration") lcd_gas = value + " ppm";
      else if (data_type == "bpm") lcd_bpm = value + " bpm";
      else if (data_type == "SpO2") lcd_spo2 = value + " %";
    }

    if (end == -1) break;
    start = end + 1;
  }
}


void displayScreen() { // Funcion que imprime valores de sensores en LCD que llegan de la base de datos
  lcd.clear();

  if (screenIndex == 0) {             // Muestra valor de temperatura
    lcd.setCursor(0, 0);
    lcd.print("Temperatura:");
    lcd.setCursor(0, 1);
    lcd.print(lcd_temperature);
  } else if (screenIndex == 1) {       // Muestra valor de humedad    
    lcd.setCursor(0, 0);
    lcd.print("Humedad:");
    lcd.setCursor(0, 1);
    lcd.print(lcd_humidity);
  } else if (screenIndex == 2) {      // Muestra concentración de CO
    lcd.setCursor(0, 0);
    lcd.print("CO:");
    lcd.setCursor(0, 1);
    lcd.print(lcd_gas);
  } else if (screenIndex == 3) {      // Muestra la frecuencia cardíaca
    lcd.setCursor(0, 0);
    lcd.print("Pulso cardiaco:");
    lcd.setCursor(0, 1);
    lcd.print(lcd_bpm);
  } else if (screenIndex == 4) {      // Muestra el nivel de oxígeno en sangre
    lcd.setCursor(0, 0);
    lcd.print("Oxigeno (SpO2):");
    lcd.setCursor(0, 1);
    lcd.print(lcd_spo2);
  }

  screenIndex = (screenIndex + 1) % 5;     // Cambia al siguiente valor a mostrar (ciclo de 0 a 4)
}

///////////////////////////////// FUNCIONES FreeRTOS ////////////////////////////////////////////////////////

void TaskMAX30105(void *pvParameters) {
  for (;;) {
    MAX30105_read(); // Lectura del MAX30105 (SpO2 y BPM)

    if (newMAX30105Data) {
      MQTT_publish_max30105();  // Publicar valores del sensor
      newMAX30105Data = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Espera entre lecturas 
  }
}

//////////////////////////// SETUP & LOOP /////////////////////////////////////////////////////////////

void setup() { 
  Serial.begin(115200);     // Inicializa la comunicación serial a 115200 baudios
  
  MQ135_setup();            // Inicializa el sensor de calidad del aire MQ135  
  dht.begin();              // Inicializa el sensor de temperatura y humedad DHT
  MAX30105_setup();         // Inicializa el sensor de pulso y oxígeno MAX30105

  lcd.begin(16,2);          // Inicializa la pantalla LCD de 16 columnas y 2 filas
  lcd.clear();
  lcd.print("Comenzando...");
  delay(1000);

  wifiMulti.addAP(ssid, password);      // red y clave de internet casa (estos datos se modifican en el archivo data.h)
  wifiMulti.addAP(ssid2, password2);    // red y clave de internet UP (estos datos se modifican en el archivo data.h)
  WiFi.mode(WIFI_STA);                      // Configura el WiFi en modo estación (cliente)

  clienteMQTT.begin(BrokerMQTT, net);       // Inicializa el cliente MQTT
  clienteMQTT.onMessage(callback);          // Recepcion de mensaje MQTT
  conectar();                               //Conexion MQTT y Wifi
  clienteMQTT.subscribe(suscribe_topic);    //Suscripcion a MQTT topic

  
  Serial.println();

  xTaskCreatePinnedToCore(        // Crea tarea para sensor MAX30105Task
    TaskMAX30105,
    "MAX30105Task",
    4096,
    NULL,
    1,
    &TaskMAX30105Handle,
    1
  );

}

void loop() {

  // Parte que controla la pausa y reanudacion del FreeRTOS
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' && lecturaActiva) {
      vTaskSuspend(TaskMAX30105Handle);
      lecturaActiva = false;
      Serial.println("Lectura MAX30105 pausada.");
    } else if (c == 'r' && !lecturaActiva) {
      vTaskResume(TaskMAX30105Handle);
      lecturaActiva = true;
      Serial.println("Lectura MAX30105 reanudada.");
    }
  }

  clienteMQTT.loop();

  if (millis() - lastUpdateTime > updateInterval) { // Actualiza la pantalla cada cierto tiempo (definido en updateInterval)
    displayScreen();
    lastUpdateTime = millis();
  }

  unsigned long currentMillis = millis();
  
  if(currentMillis - lastTempHumRead >= tempHumInterval){ // Lectura de Temp & Humedad cada cierto tiempo (definido en tempHumInterval)
    lastTempHumRead = currentMillis;
    DHT11_read();            
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C ");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" % ");
    newTempHumData = true;  
  }

  if (currentMillis - lastCORead >= coInterval) { // Lectura de CO2 cada cierto tiempo (definido en coInterval)
    lastCORead = currentMillis;
    MQ135_read();          
    Serial.print("CO Concentracion: ");
    Serial.print(gas);
    Serial.println(" ppm");  
    newCO2Data = true;     
  }

  if (newTempHumData) {
    MQTT_publish_temperature_humidity();  // Publicar temperatura y humedad
    newTempHumData = false;
  }

  if (newCO2Data) {
    MQTT_publish_co();  // Publica concentracion de CO2
    newCO2Data = false;
  }

}


////////////////////////////////////////////////////////////////////////////////////////////////////////
