#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "Adafruit_VEML7700.h"
#include "Adafruit_SHT31.h"
#include <SparkFun_AS7343.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <SensirionI2cScd4x.h>

// --- NEW: DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>

// =========================
// WiFi / MQTT CONFIG
// =========================
const char* ssid = "hydroleaf";
const char* password = "Reza11Reza11";

const char* mqtt_server = "192.168.8.3";
const int mqtt_port = 1883;
const char* mqtt_topic  = "growSensors";   // telemetry topic (not retained)

// =========================
// DEVICE METADATA
// =========================
const char* DEVICE_ID = "G01";
const char* LAYER     = "L01";
const char* SYSTEM_ID = "S01";

// =========================
// PINS
// =========================
#define I2C_SDA 18        // <-- your SDA
#define I2C_SCL 22        // <-- your SCL
#define ONE_WIRE_PIN 17   // <-- DS18B20 data (with 4.7k pull-up to 3.3V)

// =========================
// GLOBALS & OBJECTS
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_VEML7700 veml7700;
Adafruit_SHT31 sht3x = Adafruit_SHT31();
SfeAS7343ArdI2C as7343;
SensirionI2cScd4x scd41;              // NEW: CO2 sensor

// NEW: DS18B20
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
bool ds18Online = false;

bool sht3xOnline = false;
bool vemlOnline = false;
bool as7343Online = false;
bool scd41Online  = false;

// Diagnostics
unsigned long failedConnections = 0;
unsigned long failedSends = 0;
unsigned long disconnectedCounter = 0;

// CO2 state (last valid reading)
static uint16_t lastCo2Ppm = 0;
static bool     co2HasValue = false;
static float    scd41Temp = NAN, scd41Hum = NAN;

// =========================
// HELPERS
// =========================
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

// WiFi connect
void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

// Time sync
void setup_time() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP");
  while (time(nullptr) < 100000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nTime synced.");
}

// ISO8601 time
String getTimeISO8601() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", t);
  return String(buf);
}

template<typename T>
bool isValidNumber(T v) { return !isnan(v) && !isinf(v); }

String buildCompositeId(const char* systemId, const char* layer, const char* deviceId) {
  String s(systemId); s += "-"; s += layer; s += "-"; s += deviceId;
  return s;
}

void mqttReconnect() {
  while (!client.connected()) {
    String cid = String("ESP32-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Connecting to MQTT...");
    if (client.connect(cid.c_str())) {
      Serial.println("connected.");
    } else {
      Serial.print("❌ MQTT failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2s");
      failedConnections++;
      delay(2000);
    }
  }
}

void addSensor(JsonArray& arr,
               const char* sensorName,
               const char* sensorType,
               float value,
               const char* unit) {
  if (!isValidNumber(value)) return;
  JsonObject o = arr.createNestedObject();
  o["sensorName"] = sensorName;
  o["sensorType"]  = sensorType;
  o["value"]      = value;
  o["unit"]       = unit;
}

// build telemetry payload  **** CHANGED: added scdOnline & co2ppm ****
size_t buildPayload(char* out, size_t cap,
                    const char* systemId,
                    const char* deviceId,
                    const char* layer,
                    const String& timestamp,
                    bool shtOnline, bool vemlOnlineNow, bool asOnline, bool scdOnline, bool dsOnline,
                    float temp, float hum, float lux, float co2ppm, float dsTempC,
                    const uint16_t ch[14]) {

  StaticJsonDocument<4096> doc;

  // top-level metadata
  doc["system"]      = systemId;                                 // "Sxx"
  doc["layer"]       = layer;                                    // "Lxx"
  doc["deviceId"]    = deviceId;                                 // "Gxx"
  doc["compositeId"] = buildCompositeId(systemId, layer, deviceId); // "Sxx-Lxx-Gxx"
  doc["timestamp"]   = timestamp;                // ISO-8601

  // sensors[]
  JsonArray sensors = doc.createNestedArray("sensors");

  // SHT3x
  if (shtOnline) {
    addSensor(sensors, "SHT3x",   "temperature", temp, "°C");
    addSensor(sensors, "SHT3x",   "humidity",    hum,  "%");
  }

  // VEML7700
  if (vemlOnlineNow) {
    addSensor(sensors, "VEML7700","light",       lux,  "lux");
  }

  // SCD41 (CO2) — always include when device is online; value may be last known
  if (scdOnline) {
    addSensor(sensors, "SCD41", "co2", co2ppm, "ppm");
  }
  if (dsOnline) {
    addSensor(sensors, "DS18B20", "temperature", dsTempC, "°C");
  }
  if (asOnline) {
    const char* bands[11] = {
      "405nm","425nm","450nm","475nm","515nm","550nm","555nm","600nm","640nm","690nm","745nm"
    };
    for (int i = 0; i < 11; i++) addSensor(sensors, "AS7343", bands[i], ch[i], "raw");
    addSensor(sensors, "AS7343", "VIS1", ch[11], "raw");
    addSensor(sensors, "AS7343", "VIS2", ch[12], "raw");
    addSensor(sensors, "AS7343", "NIR855",    ch[13], "raw");
  }

  JsonObject health = doc.createNestedObject("health");
  health["sht3x"]    = shtOnline;
  health["veml7700"] = vemlOnlineNow;
  health["as7343"]   = asOnline;
  health["scd41"]    = scdOnline;
  health["ds18b20"]  = dsOnline;

  return serializeJson(doc, out, cap);
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  // I2C on your chosen pins
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());

  setup_wifi();
  setup_time();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(30);
  client.setBufferSize(2048);

  // Probe & init sensors
  sht3xOnline = checkI2CDevice(0x44) && sht3x.begin(0x44);
  vemlOnline = checkI2CDevice(0x10) && veml7700.begin();

  if (checkI2CDevice(0x39) && as7343.begin()) {
    as7343.powerOn(true);
    as7343.setAutoSmux(AUTOSMUX_18_CHANNELS);
    as7343.enableSpectralMeasurement(true);
    as7343Online = true;
  }

  // Init SCD41 (CO2) @ 0x62   **** NEW ****
  scd41Online = checkI2CDevice(0x62);
  if (scd41Online) {
    scd41.begin(Wire, 0x62);
    scd41.stopPeriodicMeasurement();
    delay(500);
    // scd41.setAutomaticSelfCalibration(true); // optional
    scd41.startPeriodicMeasurement();
  }

  // DS18B20
  ds18b20.begin();
  ds18Online = (ds18b20.getDeviceCount() > 0);
}

// =========================
/* LOOP */
// =========================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    disconnectedCounter++;
    Serial.printf("WiFi disconnected (%lu). Reconnecting...\n", disconnectedCounter);
    setup_wifi();
  }
  if (!client.connected()) mqttReconnect();
  client.loop();

  bool shtOnline      = checkI2CDevice(0x44) && sht3xOnline;
  bool vemlOnlineNow  = checkI2CDevice(0x10) && vemlOnline;
  bool asOnline       = checkI2CDevice(0x39) && as7343Online;
  bool scdOnline      = checkI2CDevice(0x62) && scd41Online;
  bool dsOnlineNow    = ds18Online; // OneWire devices معمولاً hot-plug تشخیص نمی‌دن

  String timestamp = getTimeISO8601();

  float temp = NAN, hum = NAN, lux = NAN;
  uint16_t ch[14] = {0};
  float dsTempC = NAN;

  if (shtOnline) {
    temp = sht3x.readTemperature();
    hum = sht3x.readHumidity();
  }
  if (vemlOnlineNow) {
    lux = veml7700.readLux();
  }
  if (asOnline) {
    as7343.readSpectraDataFromSensor();
    ch[0]  = as7343.getChannelData(CH_PURPLE_F1_405NM);
    ch[1]  = as7343.getChannelData(CH_DARK_BLUE_F2_425NM);
    ch[2]  = as7343.getChannelData(CH_BLUE_FZ_450NM);
    ch[3]  = as7343.getChannelData(CH_LIGHT_BLUE_F3_475NM);
    ch[4]  = as7343.getChannelData(CH_BLUE_F4_515NM);
    ch[5]  = as7343.getChannelData(CH_GREEN_F5_550NM);
    ch[6]  = as7343.getChannelData(CH_GREEN_FY_555NM);
    ch[7]  = as7343.getChannelData(CH_ORANGE_FXL_600NM);
    ch[8]  = as7343.getChannelData(CH_BROWN_F6_640NM);
    ch[9]  = as7343.getChannelData(CH_RED_F7_690NM);
    ch[10] = as7343.getChannelData(CH_DARK_RED_F8_745NM);
    ch[11] = as7343.getChannelData(CH_VIS_1);
    ch[12] = as7343.getChannelData(CH_VIS_2);
    ch[13] = as7343.getChannelData(CH_NIR_855NM);
  }

  if (scdOnline) {
    bool ready = false;
    if (scd41.getDataReadyStatus(ready) == 0 && ready) {
      uint16_t ppm = 0;
      float tC = 0, rH = 0;
      if (scd41.readMeasurement(ppm, tC, rH) == 0 && ppm > 0) {
        lastCo2Ppm = ppm;
        scd41Temp  = tC;
        scd41Hum   = rH;
        co2HasValue = true;
      }
    }
  }

  // DS18B20 read
  if (dsOnlineNow) {
    ds18b20.requestTemperatures();
    dsTempC = ds18b20.getTempCByIndex(0); // اولی از روی باس
  }

  char buffer[4096];
  float co2Value = (scdOnline && co2HasValue) ? (float)lastCo2Ppm : 0.0f;

  size_t n = buildPayload(buffer, sizeof(buffer),
                          SYSTEM_ID, DEVICE_ID, LAYER,
                          timestamp,
                          shtOnline, vemlOnlineNow, asOnline, scdOnline, dsOnlineNow,
                          temp, hum, lux, co2Value, dsTempC,
                          ch);

  Serial.printf("Payload length: %u\n", (unsigned)n);
  bool ok = client.publish(mqtt_topic, (uint8_t*)buffer, n, false);
  if (ok) {
    Serial.println("✅ MQTT publish OK");
    Serial.println(buffer);
  } else {
    Serial.println("❌ MQTT publish failed!");
    failedSends++;
  }

  Serial.printf("Stats: disconnects=%lu, connFails=%lu, sendFails=%lu\n",
                disconnectedCounter, failedConnections, failedSends);

  delay(1000);
}
