#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_VEML7700.h>
#include <SparkFun_AS7343.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <SensirionI2cScd4x.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include <esp_task_wdt.h>

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
const char* DEVICE_ID = "C02";
const char* LAYER     = "L02";
const char* SYSTEM_ID = "S01";

// =========================
// I2C PINS
// =========================
#define I2C_SDA 8
#define I2C_SCL 9

// =========================
// GLOBALS & OBJECTS
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_VEML7700   veml7700;
Adafruit_SHT31      sht31;
SensirionI2cScd4x   scd4x;
SfeAS7343ArdI2C     as7343;

bool vemlOnline = false;
bool shtOnline  = false;
bool scdOnline  = false;
bool asOnline   = false;

uint8_t shtAddr           = 0x44;  // SHT3x address
const uint8_t AS7343_ADDR = 0x39;  // AS7343 address
float last_co2 = NAN;              // cache CO2

unsigned long failedConnections = 0;
unsigned long failedSends       = 0;
unsigned long disconnectedCounter = 0;

// ---- large buffers on heap/global ----
DynamicJsonDocument JSON_DOC(6144);
char MQTT_BUFFER[4096];

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
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void setup_time() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP");
  while (time(nullptr) < 100000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nTime synced.");
}

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

bool mqttReconnect(uint32_t msTimeout = 8000) {
  uint32_t t0 = millis();
  while (!client.connected() && (millis() - t0) < msTimeout) {
    String cid = String("ESP32-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(cid.c_str())) {
      Serial.println("MQTT connected.");
      return true;
    }
    delay(500);
  }
  return client.connected();
}

void addSensor(JsonArray& arr,
               const char* sensorName,
               const char* sensorType,
               float value,
               const char* unit) {
  if (!isValidNumber(value)) return; // skip NaN/inf
  JsonObject o = arr.createNestedObject();
  o["sensorName"] = sensorName;
  o["sensorType"] = sensorType;
  o["value"]      = value;
  o["unit"]       = unit;
}

// =========================
// PAYLOAD BUILDER  (SCD41 -> only CO2)
// =========================
size_t buildPayload(char* out, size_t cap,
                    const char* systemId,
                    const char* deviceId,
                    const char* layer,
                    const String& timestamp,
                    bool vemlOnlineNow, float lux,
                    bool shtOnlineNow,  float tC, float rh,
                    bool scdOnlineNow,  float co2ppm,
                    bool asOnlineNow,   const uint16_t* f, size_t fCount,
                    int extraCh13, int extraCh14) {

  JSON_DOC.clear();
  JSON_DOC["system"]      = systemId;
  JSON_DOC["layer"]       = layer;
  JSON_DOC["deviceId"]    = deviceId;
  JSON_DOC["compositeId"] = buildCompositeId(systemId, layer, deviceId);
  JSON_DOC["timestamp"]   = timestamp;

  JsonArray sensors = JSON_DOC.createNestedArray("sensors");

  if (vemlOnlineNow) addSensor(sensors, "VEML7700", "light", lux, "lux");

  if (shtOnlineNow) {
    addSensor(sensors, "SHT3x", "A_Temp_C", tC, "C");
    addSensor(sensors, "SHT3x", "A_RH_C",   rh, "%");
  }

  if (scdOnlineNow) {
    addSensor(sensors, "SCD41", "co2", co2ppm, "ppm");
  }

  if (asOnlineNow && f && fCount >= 12) {
    const char* nm[12] = {
      "405nm","425nm","475nm","515nm","550nm","640nm",
      "690nm","745nm","450nm","555nm","600nm","855nm"
    };
    for (int i = 0; i < 12; ++i)
      addSensor(sensors, "AS7343", nm[i], (float)f[i], "counts");
  }

  // Extra spectral channels (13 = VIS1, 14 = FD1)
  if (asOnlineNow) {
    if (extraCh13 >= 0) addSensor(sensors, "AS7343", "VIS1", (float)extraCh13, "counts");
    if (extraCh14 >= 0) addSensor(sensors, "AS7343", "FD1",  (float)extraCh14, "counts");
  }

  JsonObject health = JSON_DOC.createNestedObject("health");
  health["veml7700"] = vemlOnlineNow;
  health["sht3x"]    = shtOnlineNow;
  health["scd41"]    = scdOnlineNow;
  health["as7343"]   = asOnlineNow;

  return serializeJson(JSON_DOC, out, cap);
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(1500);
  while (!Serial) { delay(10); }

  esp_task_wdt_deinit(); // disable watchdog logs

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());

  setup_wifi();
  setup_time();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(30);
  client.setBufferSize(sizeof(MQTT_BUFFER));

  // Probes
  vemlOnline = checkI2CDevice(0x10) && veml7700.begin();
  shtOnline  = checkI2CDevice(shtAddr) && sht31.begin(shtAddr);

  if (checkI2CDevice(0x62)) {
    scd4x.begin(Wire, 0x62);
    scd4x.stopPeriodicMeasurement();
    delay(20);
    scd4x.setAutomaticSelfCalibrationEnabled(true);
    scd4x.startPeriodicMeasurement(); // ~5s per sample
    scdOnline = true;
  }

  if (checkI2CDevice(AS7343_ADDR)) {
    if (as7343.begin(AS7343_ADDR, Wire)) {
      asOnline = true;

      // ✅ Key to prevent zero output:
      as7343.powerOn(true);                          // PON=1
      as7343.enableSpectralMeasurement(true);        // SP_EN=1
      as7343.setAutoSmux(AUTOSMUX_18_CHANNELS);      // 18-Channel Cycle (F + VIS/FD)

      // (Optional: Lower rate/simplified if you want)
      // as7343.setAutoSmux(AUTOSMUX_12_CHANNELS);
    }
  }
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

  bool vemlOnlineNow = vemlOnline && checkI2CDevice(0x10);
  bool shtOnlineNow  = shtOnline  && checkI2CDevice(shtAddr);
  bool scdOnlineNow  = scdOnline; // periodic mode keeps running internally
  bool asOnlineNow   = asOnline   && checkI2CDevice(AS7343_ADDR);

  String timestamp = getTimeISO8601();

  // --- VEML7700 ---
  float lux = NAN;
  if (vemlOnlineNow) lux = veml7700.readLux();

  // --- SHT3x ---
  float tC = NAN, rh = NAN;
  if (shtOnlineNow) {
    tC = sht31.readTemperature();
    rh = sht31.readHumidity();
    if (!isValidNumber(tC) || tC < -40 || tC > 125) tC = NAN;
    if (!isValidNumber(rh) || rh < 0   || rh > 100) rh = NAN;
  }

  // --- SCD41 (CO2 only) ---
  float co2ppm = last_co2; // send last known if no new data
  if (scdOnlineNow) {
    bool dataReady = false;
    if (scd4x.getDataReadyStatus(dataReady) == 0 && dataReady) {
      uint16_t co2Raw; float tDummy, rhDummy;
      if (scd4x.readMeasurement(co2Raw, tDummy, rhDummy) == 0) {
        last_co2 = (float)co2Raw;
        co2ppm   = last_co2;
      }
    }
  }

  // --- AS7343 (F1..F12 + ch13/ch14) ---
  uint16_t f[12] = {0};
  int extra13 = -1; // VIS_1
  int extra14 = -1; // FD_1
  if (asOnlineNow) {
    // Read all the channels at once after the measurement is enabled.
    as7343.readSpectraDataFromSensor();

    // F1..F12 mapped to library enum names
    f[0]  = as7343.getChannelData(CH_PURPLE_F1_405NM);
    f[1]  = as7343.getChannelData(CH_DARK_BLUE_F2_425NM);
    f[2]  = as7343.getChannelData(CH_LIGHT_BLUE_F3_475NM);
    f[3]  = as7343.getChannelData(CH_BLUE_F4_515NM);
    f[4]  = as7343.getChannelData(CH_GREEN_F5_550NM);
    f[5]  = as7343.getChannelData(CH_BROWN_F6_640NM);
    f[6]  = as7343.getChannelData(CH_RED_F7_690NM);
    f[7]  = as7343.getChannelData(CH_DARK_RED_F8_745NM);
    f[8]  = as7343.getChannelData(CH_BLUE_FZ_450NM);
    f[9]  = as7343.getChannelData(CH_GREEN_FY_555NM);
    f[10] = as7343.getChannelData(CH_ORANGE_FXL_600NM);
    f[11] = as7343.getChannelData(CH_NIR_855NM);

    // Channel 13 & 14
    extra13 = (int)as7343.getChannelData(CH_VIS_1); // VIS1
    extra14 = (int)as7343.getChannelData(CH_FD_1);  // FD1
  }

  // ---------- Build & publish ----------
  size_t n = buildPayload(MQTT_BUFFER, sizeof(MQTT_BUFFER),
                          SYSTEM_ID, DEVICE_ID, LAYER, timestamp,
                          vemlOnlineNow, lux,
                          shtOnlineNow,  tC, rh,
                          scdOnlineNow,  co2ppm,
                          asOnlineNow,   f, 12,
                          extra13, extra14);

  // Print final JSON to Serial
  Serial.println("---- JSON ----");
  Serial.write(MQTT_BUFFER, n);
  Serial.println();
  Serial.println("--------------");

  bool ok = client.publish(mqtt_topic, (uint8_t*)MQTT_BUFFER, n, false);
  if (ok) {
    Serial.println("✅ MQTT publish OK");
  } else {
    Serial.println("❌ MQTT publish failed!");
    failedSends++;
  }

  delay(1000); //AS7343/CO2 are updated every ~few cycles;we send the last quantity
}
