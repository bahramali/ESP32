/*
  esp_growSensors.ino
  - ESP32 + VEML7700 + SHT3x + SCD41 + AS7343
  - AS7343 smoothing: boxcar average + EMA
  - AS7343 anti-saturation: hysteresis + cooldown (no getAgain/ATIME/ASTEP)
  - Publishes JSON to MQTT
*/

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
#include <string.h> // for memcpy

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
const char* DEVICE_ID = "C01";
const char* LAYER     = "L01";
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

uint8_t shtAddr           = 0x44;  // SHT3x I2C address
const uint8_t AS7343_ADDR = 0x39;  // AS7343 I2C address
float last_co2 = NAN;

unsigned long failedConnections = 0;
unsigned long failedSends       = 0;
unsigned long disconnectedCounter = 0;

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
  if (!isValidNumber(value)) return;
  JsonObject o = arr.createNestedObject();
  o["sensorName"] = sensorName;
  o["sensorType"] = sensorType;
  o["value"]      = value;
  o["unit"]       = unit;
}

// =========================
// JSON Payload builder
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

  // NOTE: sensorType in UPPERCASE except nm channels
  if (vemlOnlineNow) addSensor(sensors, "VEML7700", "LUX", lux, "lux");

  if (shtOnlineNow) {
    addSensor(sensors, "SHT3x", "A_TEMP_C", tC, "C");
    addSensor(sensors, "SHT3x", "A_RH_C",   rh, "%");
  }

  if (scdOnlineNow) {
    addSensor(sensors, "SCD41", "CO2", co2ppm, "ppm");
  }

  if (asOnlineNow && f && fCount >= 12) {
    // nm strings stay as-is (digits + "nm")
    const char* nm[12] = {
      "405nm","425nm","475nm","515nm","550nm","640nm",
      "690nm","745nm","450nm","555nm","600nm","855nm"
    };
    for (int i = 0; i < 12; ++i)
      addSensor(sensors, "AS7343", nm[i], (float)f[i], "counts");
  }

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
// AS7343 smoothing & anti-saturation
// (no getAgain/ATIME/ASTEP; we track gain code locally)
// =========================

// --- Gain control bounds & state ---
int AS_GAIN_MIN_CODE = 0;   // conservative lower bound (usually 0)
int AS_GAIN_MAX_CODE = 7;   // conservative upper bound (often 7 => 64X)
int as_currentGainCode = 2; // starting gain (≈2X)

// --- UI clamp thresholds (0..999 UI scale on your side) ---
const uint16_t AS_MAX_THR = 900; // near saturation
const uint16_t AS_MIN_THR = 250; // too low

// --- Gain change hysteresis/cooldown ---
const int GAIN_COOLDOWN_CYCLES = 20; // wait N loops after each change
int gainCooldown = 0;

void as7343_setGainCode(int code) {
  if (code < AS_GAIN_MIN_CODE) code = AS_GAIN_MIN_CODE;
  if (code > AS_GAIN_MAX_CODE) code = AS_GAIN_MAX_CODE;
  as_currentGainCode = code;
  as7343.setAgain((sfe_as7343_again_t)as_currentGainCode);
  gainCooldown = GAIN_COOLDOWN_CYCLES; // start cooldown
  delay(2);
}

// --- Multi-sample average + EMA ---
#define AS_SAMPLES 9        // boxcar window per publish (smoother)
#define EMA_ALPHA  0.12f    // 0..1 ; lower = smoother

bool   emaInit = false;
float  emaF[12] = {0};      // EMA per channel
float  emaCh13 = 0, emaCh14 = 0;

void readAveragedSpectra(uint16_t outF[12], int &ch13, int &ch14) {
  // Discard first read (after any possible change)
  as7343.readSpectraDataFromSensor();
  delay(2);

  uint32_t acc[12] = {0};
  uint32_t acc13 = 0, acc14 = 0;

  for (int s = 0; s < AS_SAMPLES; ++s) {
    as7343.readSpectraDataFromSensor();

    acc[0]  += as7343.getChannelData(CH_PURPLE_F1_405NM);
    acc[1]  += as7343.getChannelData(CH_DARK_BLUE_F2_425NM);
    acc[2]  += as7343.getChannelData(CH_LIGHT_BLUE_F3_475NM);
    acc[3]  += as7343.getChannelData(CH_BLUE_F4_515NM);
    acc[4]  += as7343.getChannelData(CH_GREEN_F5_550NM);
    acc[5]  += as7343.getChannelData(CH_BROWN_F6_640NM);
    acc[6]  += as7343.getChannelData(CH_RED_F7_690NM);
    acc[7]  += as7343.getChannelData(CH_DARK_RED_F8_745NM);
    acc[8]  += as7343.getChannelData(CH_BLUE_FZ_450NM);
    acc[9]  += as7343.getChannelData(CH_GREEN_FY_555NM);
    acc[10] += as7343.getChannelData(CH_ORANGE_FXL_600NM);
    acc[11] += as7343.getChannelData(CH_NIR_855NM);

    acc13   += as7343.getChannelData(CH_VIS_1);
    acc14   += as7343.getChannelData(CH_FD_1);

    delay(15); // spread samples across mains-flicker
  }

  for (int i = 0; i < 12; ++i) outF[i] = (uint16_t)(acc[i] / AS_SAMPLES);
  ch13 = (int)(acc13 / AS_SAMPLES);
  ch14 = (int)(acc14 / AS_SAMPLES);
}

void applyEMA(uint16_t inF[12], int in13, int in14,
              uint16_t outF[12], int &out13, int &out14) {
  if (!emaInit) {
    for (int i = 0; i < 12; ++i) emaF[i] = (float)inF[i];
    emaCh13 = (float)in13; emaCh14 = (float)in14;
    emaInit = true;
  } else {
    for (int i = 0; i < 12; ++i)
      emaF[i] = EMA_ALPHA * (float)inF[i] + (1.0f - EMA_ALPHA) * emaF[i];
    emaCh13 = EMA_ALPHA * (float)in13 + (1.0f - EMA_ALPHA) * emaCh13;
    emaCh14 = EMA_ALPHA * (float)in14 + (1.0f - EMA_ALPHA) * emaCh14;
  }
  for (int i = 0; i < 12; ++i) outF[i] = (uint16_t)(emaF[i] + 0.5f);
  out13 = (int)(emaCh13 + 0.5f);
  out14 = (int)(emaCh14 + 0.5f);
}

void as7343_antiSaturationOnce() {
  // Quick probe (single read) to check peak before averaging
  as7343.readSpectraDataFromSensor();
  uint16_t mx = 0;
  uint16_t probe[12] = {
    as7343.getChannelData(CH_PURPLE_F1_405NM),
    as7343.getChannelData(CH_DARK_BLUE_F2_425NM),
    as7343.getChannelData(CH_LIGHT_BLUE_F3_475NM),
    as7343.getChannelData(CH_BLUE_F4_515NM),
    as7343.getChannelData(CH_GREEN_F5_550NM),
    as7343.getChannelData(CH_BROWN_F6_640NM),
    as7343.getChannelData(CH_RED_F7_690NM),
    as7343.getChannelData(CH_DARK_RED_F8_745NM),
    as7343.getChannelData(CH_BLUE_FZ_450NM),
    as7343.getChannelData(CH_GREEN_FY_555NM),
    as7343.getChannelData(CH_ORANGE_FXL_600NM),
    as7343.getChannelData(CH_NIR_855NM)
  };
  for (int i=0;i<12;++i) if (probe[i] > mx) mx = probe[i];

  if (gainCooldown > 0) { gainCooldown--; return; }

  if (mx > AS_MAX_THR && as_currentGainCode > AS_GAIN_MIN_CODE) {
    as7343_setGainCode(as_currentGainCode - 1); // decrease gain
  } else if (mx < AS_MIN_THR && as_currentGainCode < AS_GAIN_MAX_CODE) {
    as7343_setGainCode(as_currentGainCode + 1); // increase gain
  }
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

      as7343.powerOn(true);
      as7343.enableSpectralMeasurement(true);
      as7343.setAutoSmux(AUTOSMUX_18_CHANNELS);

      // Conservative starting gain & discard first sample
      as7343_setGainCode(as_currentGainCode); // apply starting gain code (2)
      delay(50);
      as7343.readSpectraDataFromSensor(); // throw away first read
      delay(5);
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
  bool scdOnlineNow  = scdOnline; // periodic mode keeps running
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

  // --- AS7343 ---
  uint16_t f[12] = {0};
  int extra13 = -1; // VIS1
  int extra14 = -1; // FD1
  if (asOnlineNow) {
    // Gain adjust sparsely (with cooldown)
    as7343_antiSaturationOnce();

    // Average multiple samples
    uint16_t f_raw[12] = {0};
    int ch13_raw = -1, ch14_raw = -1;
    readAveragedSpectra(f_raw, ch13_raw, ch14_raw);

    // EMA smoothing
    uint16_t f_pub[12] = {0};
    int ch13_pub = -1, ch14_pub = -1;
    applyEMA(f_raw, ch13_raw, ch14_raw, f_pub, ch13_pub, ch14_pub);

    // Use smoothed data for publishing
    memcpy(f, f_pub, sizeof(f_pub));
    extra13 = ch13_pub;
    extra14 = ch14_pub;
  }

  // ---------- Build & publish ----------
  size_t n = buildPayload(MQTT_BUFFER, sizeof(MQTT_BUFFER),
                          SYSTEM_ID, DEVICE_ID, LAYER, timestamp,
                          vemlOnlineNow, lux,
                          shtOnlineNow,  tC, rh,
                          scdOnlineNow,  co2ppm,
                          asOnlineNow,   f, 12,
                          extra13, extra14);

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

  delay(1000);
}