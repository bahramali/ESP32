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
// GLOBAL OBJECTS
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_VEML7700  veml7700;
Adafruit_SHT31     sht3x = Adafruit_SHT31();
SfeAS7343ArdI2C    as7343;        // SparkFun AS7343 (I2C)
SensirionI2cScd4x  scd41;         // Sensirion SCD41 (CO₂)

bool sht3xOnline = false;
bool vemlOnline  = false;
bool as7343Online= false;
bool scd41Online = false;

// Diagnostic counters
unsigned long failedConnections = 0;
unsigned long failedSends = 0;
unsigned long disconnectedCounter = 0;

// CO₂ last valid reading
static uint16_t lastCo2Ppm = 0;
static bool     co2HasValue = false;

// =========================
// HELPER FUNCTIONS
// =========================
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

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

void addSensor(JsonArray& arr, const char* sensorName, const char* sensorType, float value, const char* unit) {
  if (!isValidNumber(value)) return;
  JsonObject o = arr.createNestedObject();
  o["sensorName"] = sensorName;
  o["sensorType"] = sensorType;
  o["value"]      = value;
  o["unit"]       = unit;
}

// =========================
// BUILD PAYLOAD
// =========================
size_t buildPayload(char* out, size_t cap,
                    const char* systemId,
                    const char* deviceId,
                    const char* layer,
                    const String& timestamp,
                    bool shtOnline, bool vemlOnlineNow, bool asOnline, bool scdOnline,
                    float temp, float hum, float lux, float co2ppm,
                    const uint16_t ch[14]) {

  StaticJsonDocument<8192> doc;

  // ---- Metadata
  doc["system"]      = systemId;
  doc["layer"]       = layer;
  doc["deviceId"]    = deviceId;
  doc["compositeId"] = buildCompositeId(systemId, layer, deviceId);
  doc["timestamp"]   = timestamp;

  // ---- Sensors
  JsonArray sensors = doc.createNestedArray("sensors");

  // SHT3x temperature + humidity
  if (shtOnline) {
    addSensor(sensors, "SHT3x", "temperature", temp, "°C");
    addSensor(sensors, "SHT3x", "humidity",    hum,  "%");
  }

  // CO₂ (SCD41)
  if (scdOnline) {
    addSensor(sensors, "SCD41", "co2", co2ppm, "ppm");
  }

  // AS7343 – all 14 channels as separate entries
  if (asOnline) {
    const char* bands[14] = {
      "405nm","425nm","450nm","475nm","515nm","550nm","555nm",
      "600nm","640nm","690nm","745nm","VIS1","VIS2","NIR855"
    };
    for (int i = 0; i < 14; i++) {
      addSensor(sensors, "AS7343", bands[i], (float)ch[i], "raw");
    }
  }

  // VEML7700 light intensity
  if (vemlOnlineNow) {
    addSensor(sensors, "VEML7700", "light", lux, "lux");
  }

  // ---- Health flags
  JsonObject health = doc.createNestedObject("health");
  health["sht3x"]    = shtOnline;
  health["veml7700"] = vemlOnlineNow;
  health["as7343"]   = asOnline;
  health["scd41"]    = scdOnline;

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
  client.setBufferSize(8192); // large enough for full JSON payload

  // ---- Initialize SHT3x
  sht3xOnline = checkI2CDevice(0x44) && sht3x.begin(0x44);

  // ---- Initialize VEML7700
  vemlOnline  = checkI2CDevice(0x10) && veml7700.begin();

  // ---- Initialize AS7343
  if (checkI2CDevice(0x39)) {
    as7343Online = as7343.begin(0x39, Wire);
    if (as7343Online) {
      as7343.powerOn(true);
      as7343.setAutoSmux(AUTOSMUX_18_CHANNELS);
      as7343.enableSpectralMeasurement(true);
      // Optional tuning for bright light:
      //as7343.setGain(AS7343_GAIN_1X);
      //as7343.setIntTime(10);
    }
  }

  // ---- Initialize SCD41 (CO₂)
  scd41Online = checkI2CDevice(0x62);
  if (scd41Online) {
    scd41.begin(Wire, 0x62);
    scd41.stopPeriodicMeasurement();
    delay(500);
    scd41.startPeriodicMeasurement();
  }
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    disconnectedCounter++;
    setup_wifi();
  }
  if (!client.connected()) mqttReconnect();
  client.loop();

  bool shtOK = checkI2CDevice(0x44) && sht3xOnline;
  bool vemlOK= checkI2CDevice(0x10) && vemlOnline;
  bool asOK  = checkI2CDevice(0x39) && as7343Online;
  bool scdOK = checkI2CDevice(0x62) && scd41Online;

  String timestamp = getTimeISO8601();

  float temp = NAN, hum = NAN, lux = NAN;
  uint16_t ch[14] = {0};

  // ---- Read SHT3x
  if (shtOK) {
    temp = sht3x.readTemperature();
    hum  = sht3x.readHumidity();
  }

  // ---- Read VEML7700
  if (vemlOK) {
    lux = veml7700.readLux();
  }

  // ---- Read AS7343 spectral data
  if (asOK) {
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

  // ---- Read SCD41 (CO₂)
  if (scdOK) {
    bool ready = false;
    if (scd41.getDataReadyStatus(ready) == 0 && ready) {
      uint16_t ppm = 0; float t_dummy = 0, r_dummy = 0;
      if (scd41.readMeasurement(ppm, t_dummy, r_dummy) == 0 && ppm > 0) {
        lastCo2Ppm  = ppm;
        co2HasValue = true;
      }
    }
  }
  float co2Value = (scdOK && co2HasValue) ? (float)lastCo2Ppm : 0.0f;

  // ---- Build and publish JSON payload
  char buffer[8192];
  size_t n = buildPayload(buffer, sizeof(buffer),
                          SYSTEM_ID, DEVICE_ID, LAYER,
                          timestamp,
                          shtOK, vemlOK, asOK, scdOK,
                          temp, hum, lux, co2Value,
                          ch);

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
