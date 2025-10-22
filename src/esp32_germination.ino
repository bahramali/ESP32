#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "Adafruit_VEML7700.h"
// Removed: #include "Adafruit_SHT31.h"
#include <SparkFun_AS7343.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <SensirionI2cScd4x.h>

// --- NEW: DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>

// --- NEW: HDC302x (Temp + Humidity) ---
#include <Adafruit_HDC302x.h>

// =========================
// WiFi / MQTT CONFIG
// =========================
const char* ssid = "hydroleaf";
const char* password = "Reza11Reza11";

const char* mqtt_server = "192.168.8.3";
const int mqtt_port = 1883;
const char* mqtt_topic  = "germinationTopic";

// =========================
// DEVICE METADATA
// =========================
const char* DEVICE_ID = "G01";
const char* LAYER     = "L01";
const char* SYSTEM_ID = "S01";

// =========================
// PINS
// =========================
#define I2C_SDA 8        // your SDA
#define I2C_SCL 9        // your SCL
#define ONE_WIRE_PIN 14  // DS18B20 data (with 4.7k pull-up to 3.3V)

// =========================
// GLOBALS & OBJECTS
// =========================
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_VEML7700 veml7700;

// NEW: HDC302x
Adafruit_HDC302x hdc302x;

// NEW: DS18B20
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);

// Online flags
bool hdcOnline   = false;
bool vemlOnline  = false;
bool ds18Online  = false;

// Diagnostics
unsigned long failedConnections = 0;
unsigned long failedSends = 0;
unsigned long disconnectedCounter = 0;

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
  o["sensorType"] = sensorType;
  o["value"]      = value;
  o["unit"]       = unit;
}

// build telemetry payload (SHT3x -> HDC302x)
size_t buildPayload(char* out, size_t cap,
                    const char* systemId,
                    const char* deviceId,
                    const char* layer,
                    const String& timestamp,
                    bool hdcOnlineNow, bool vemlOnlineNow, bool dsOnline,
                    float temp, float hum, float lux, float dsTempC) {

  StaticJsonDocument<4096> doc;

  // top-level metadata
  doc["system"]      = systemId;
  doc["layer"]       = layer;
  doc["deviceId"]    = deviceId;
  doc["compositeId"] = buildCompositeId(systemId, layer, deviceId);
  doc["timestamp"]   = timestamp;

  // sensors[]
  JsonArray sensors = doc.createNestedArray("sensors");

  // HDC302x
  if (hdcOnlineNow) {
    addSensor(sensors, "HDC302x", "temperature", temp, "°C");
    addSensor(sensors, "HDC302x", "humidity",    hum,  "%");
  }

  // VEML7700
  if (vemlOnlineNow) {
    addSensor(sensors, "VEML7700","light",       lux,  "lux");
  }

  // DS18B20
  if (dsOnline) {
    addSensor(sensors, "DS18B20", "temperature", dsTempC, "°C");
  }

  // health
  JsonObject health = doc.createNestedObject("health");
  health["hdc302x"]  = hdcOnlineNow;
  health["veml7700"] = vemlOnlineNow;
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
  vemlOnline = checkI2CDevice(0x10) && veml7700.begin();

  // HDC302x default I2C addr is 0x44 (A0 pad can change it to 0x45)
  hdcOnline = checkI2CDevice(0x44) && hdc302x.begin(0x44);
  // Optional: enable high precision (library defaults are fine)

  // DS18B20
  ds18b20.begin();
  ds18Online = (ds18b20.getDeviceCount() > 0);
}

// =========================
// LOOP
// =========================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    disconnectedCounter++;
    Serial.printf("WiFi disconnected (%lu). Reconnecting...\n", disconnectedCounter);
    setup_wifi();
  }
  if (!client.connected()) mqttReconnect();
  client.loop();

  bool hdcOnlineNow    = checkI2CDevice(0x44) && hdcOnline;
  bool vemlOnlineNow   = checkI2CDevice(0x10) && vemlOnline;
  bool dsOnlineNow     = ds18Online;

  String timestamp = getTimeISO8601();

  double temp = NAN, hum = NAN;
  float dsTempC = NAN, lux = NAN;

  // HDC302x read
  if (hdcOnlineNow) {
      temp = 0.0;
      hum = 0.0;
    // The library exposes simple read helpers:
      hdc302x.readTemperatureHumidityOnDemand(temp, hum, TRIGGERMODE_LP0);

  }

  if (vemlOnlineNow) {
    lux = veml7700.readLux();
  }

  // DS18B20 read
  if (dsOnlineNow) {
    ds18b20.requestTemperatures();
    dsTempC = ds18b20.getTempCByIndex(0);
  }

  char buffer[4096];

  size_t n = buildPayload(buffer, sizeof(buffer),
                          SYSTEM_ID, DEVICE_ID, LAYER,
                          timestamp,
                          hdcOnlineNow, vemlOnlineNow, dsOnlineNow,
                          temp, hum, lux, dsTempC);

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
