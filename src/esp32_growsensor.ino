#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "Adafruit_VEML7700.h"
#include <SparkFun_AS7343.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <SensirionI2cScd4x.h>
#include "Adafruit_SHT31.h"

// =========================
// WiFi / MQTT CONFIG
// =========================
const char* ssid = "hydroleaf";
const char* password = "Reza11Reza11";

const char* mqtt_server = "192.168.8.3";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "growSensors";

// =========================
// DEVICE METADATA
// =========================
const char* DEVICE_ID = "A01";
const char* LAYER     = "L01";
const char* SYSTEM_ID = "S01";

// =========================
// PINS
// =========================
#define I2C_SDA 8
#define I2C_SCL 9

// =========================
// GLOBALS & OBJECTS
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_SHT31    sht3x = Adafruit_SHT31();
Adafruit_VEML7700 veml7700;
SensirionI2cScd4x scd4x;
AS7343            as7343;

bool shtOnline  = false;
bool luxOnline  = false;
bool scdOnline  = false;
bool asOnline   = false;

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

void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400); Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void setup_time() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP");
  while (time(nullptr) < 100000) { delay(400); Serial.print("."); }
  Serial.println("\nTime synced.");
}

String getTimeISO8601() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", t);
  return String(buf);
}

template<typename T> bool isValidNumber(T v) { return !isnan(v) && !isinf(v); }

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
      Serial.print("❌ MQTT failed, rc="); Serial.print(client.state());
      Serial.println(" retrying in 2s");
      failedConnections++;
      delay(2000);
    }
  }
}

void addSensor(JsonArray& arr, const char* name, const char* type, float value, const char* unit) {
  if (!isValidNumber(value)) return;
  JsonObject o = arr.createNestedObject();
  o["sensorName"] = name;
  o["sensorType"] = type;
  o["value"]      = value;
  o["unit"]       = unit;
}

// =========================
// BUILD PAYLOAD
// =========================
size_t buildPayload(char* out, size_t cap,
                    const char* systemId, const char* deviceId, const char* layer,
                    const String& timestamp,
                    bool shtOK, bool luxOK, bool scdOK, bool asOK,
                    float t_sht, float rh_sht,
                    float lux,
                    float co2ppm,
                    AS7343& as) {

  StaticJsonDocument<8192> doc;

  doc["system"]      = systemId;
  doc["layer"]       = layer;
  doc["deviceId"]    = deviceId;
  doc["compositeId"] = buildCompositeId(systemId, layer, deviceId);
  doc["timestamp"]   = timestamp;

  JsonArray sensors = doc.createNestedArray("sensors");

  if (shtOK) {
    addSensor(sensors, "SHT3x", "temperature", t_sht, "°C");
    addSensor(sensors, "SHT3x", "humidity",    rh_sht, "%");
  }

  if (scdOK) {
    addSensor(sensors, "SCD41", "co2", co2ppm, "ppm");
  }

  // ---- AS7343 all channels as individual readings
  if (asOK) {
    if (as.readAllChannels()) {
      struct {
        const char* name; as7343_channel_t ch;
      } chans[] = {
        {"405nm", AS7343_CHANNEL_F1}, {"425nm", AS7343_CHANNEL_F2},
        {"450nm", AS7343_CHANNEL_F3}, {"475nm", AS7343_CHANNEL_F4},
        {"515nm", AS7343_CHANNEL_F5}, {"550nm", AS7343_CHANNEL_F6},
        {"555nm", AS7343_CHANNEL_F7}, {"600nm", AS7343_CHANNEL_F8},
        {"640nm", AS7343_CHANNEL_F9}, {"690nm", AS7343_CHANNEL_F10},
        {"745nm", AS7343_CHANNEL_F11},{"VIS1",  AS7343_CHANNEL_CLEAR},
        {"VIS2",  AS7343_CHANNEL_F12},{"NIR855",AS7343_CHANNEL_NIR}
      };

      for (auto& c : chans) {
        addSensor(sensors, "AS7343", c.name, (float)as.getChannel(c.ch), "raw");
      }
    }
  }

  JsonObject health = doc.createNestedObject("health");
  health["sht3x"]   = shtOK;
  health["veml7700"]= luxOK;
  health["as7343"]  = asOK;
  health["scd41"]   = scdOK;

  return serializeJson(doc, out, cap);
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  setup_wifi();
  setup_time();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(30);
  client.setBufferSize(8192);

  shtOnline = checkI2CDevice(0x44) && sht3x.begin(0x44);
  luxOnline = checkI2CDevice(0x10) && veml7700.begin();

  if (checkI2CDevice(0x62)) {
    scd4x.begin(Wire);
    scd4x.stopPeriodicMeasurement();
    uint16_t err = scd4x.startPeriodicMeasurement();
    scdOnline = (err == 0);
  }

  asOnline = as7343.begin(Wire);
  if (asOnline) {
    as7343.setSMUXCommand(SMUX_20_60);
    as7343.setMeasurementTime(10);
    as7343.setGain(AS7343_GAIN_1X);
  }
}

// =========================
// LOOP
// =========================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    disconnectedCounter++;
    setup_wifi();
  }
  if (!client.connected()) mqttReconnect();
  client.loop();

  bool shtOK = shtOnline && checkI2CDevice(0x44);
  bool luxOK = luxOnline && checkI2CDevice(0x10);
  bool scdOK = scdOnline && checkI2CDevice(0x62);
  bool asOK  = asOnline;

  String timestamp = getTimeISO8601();

  float t_sht=NAN, rh_sht=NAN, lux=NAN, co2ppm=NAN;

  if (shtOK) {
    t_sht = sht3x.readTemperature();
    rh_sht = sht3x.readHumidity();
  }

  if (luxOK) {
    lux = veml7700.readLux();
  }

  if (scdOK) {
    uint16_t co2; float t_dummy, rh_dummy;
    if (scd4x.readMeasurement(co2, t_dummy, rh_dummy) == 0 && co2 != 0) {
      co2ppm = co2;
    }
  }

  char buffer[8192];
  size_t n = buildPayload(buffer, sizeof(buffer),
                          SYSTEM_ID, DEVICE_ID, LAYER,
                          timestamp,
                          shtOK, luxOK, scdOK, asOK,
                          t_sht, rh_sht,
                          lux,
                          co2ppm,
                          as7343);

  bool ok = client.publish(mqtt_topic, (uint8_t*)buffer, n, false);
  if (ok) {
    Serial.println("✅ MQTT publish OK");
    Serial.println(buffer);
  } else {
    Serial.println("❌ MQTT publish failed!");
    failedSends++;
  }

  delay(1000);
}
