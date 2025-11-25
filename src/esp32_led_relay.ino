#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

/* ===== WiFi / MQTT ===== */
const char* ssid     = "hydroleaf";
const char* password = "Reza11Reza11";

const char* mqtt_server  = "192.168.8.3";
const int   mqtt_port    = 1883;
const char* TOPIC_CMD    = "actuator/led/cmd";    // subscribe
const char* TOPIC_STATUS = "actuator/led/status"; // publish (retained)

/* ===== Target filter (per device) ===== */
const char* SYSTEM_ID = "S01";
const char* DEVICE_ID = "R01";

/* ===== Relay config (4 channels) ===== */
static const int RELAY_COUNT = 4;

// Change these pins to match your ESP32-S3 board wiring
int RELAY_PINS[RELAY_COUNT] = {10, 11, 12, 13};

// Each relay controls one layer
const char* CHANNEL_LAYER[RELAY_COUNT] = {
  "L01", // relay 0 -> layer 1
  "L02", // relay 1 -> layer 2
  "L03", // relay 2 -> layer 3
  "L04"  // relay 3 -> layer 4
};

const bool RELAY_ACTIVE_HIGH = false; // set false if your module is active-low

/* ===== Schedule (configurable via MQTT JSON) ===== */
// Default: 05:00 -> ON for 16h
int gOnHour          = 5;
int gOnMinute        = 0;
int gOnSecond        = 0;
int gOnDurationHours = 16;

/* ===== Timezone (Europe/Stockholm w/ DST) ===== */
static const char* TZ_EUROPE_STOCKHOLM = "CET-1CEST,M3.5.0/2,M10.5.0/3";

/* ===== Globals ===== */
WiFiClient espClient;
PubSubClient client(espClient);

enum Mode { MODE_AUTO = 0, MODE_MANUAL = 1 };

// Per-channel state
Mode  currentMode[RELAY_COUNT];
bool  relayState[RELAY_COUNT];
bool  hasDriven[RELAY_COUNT];
time_t manualUntil[RELAY_COUNT];  // when MANUAL expires; 0 means no expiry

/* ---------- Helpers ---------- */
static bool nonEmptyEq(const char* want, const char* got) {
  if (!want || *want == '\0') return true;
  if (!got) return false;
  return String(got).equalsIgnoreCase(want);
}

int channelIndexByLayer(const char* layer) {
  if (!layer) return -1;
  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (String(layer).equalsIgnoreCase(CHANNEL_LAYER[i])) {
      return i;
    }
  }
  return -1;
}

inline void driveRelay(int idx, bool on) {
  if (idx < 0 || idx >= RELAY_COUNT) return;
  int pin = RELAY_PINS[idx];
  int level = RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
  digitalWrite(pin, level);
  relayState[idx] = on;
  hasDriven[idx]  = true;
  Serial.printf("[Relay %d L=%s] -> %s\n",
                idx, CHANNEL_LAYER[idx],
                on ? "ON" : "OFF");
}

static inline int secondsSinceMidnight(const tm& lt) {
  return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec;
}

bool timeIsValid() { return time(nullptr) > 100000; }

bool scheduleShouldBeOnNow(tm* outLt = nullptr) {
  time_t now = time(nullptr);
  if (!timeIsValid()) return false;

  tm lt;
  localtime_r(&now, &lt);
  if (outLt) *outLt = lt;

  const int startSec = gOnHour * 3600 + gOnMinute * 60 + gOnSecond;
  const int endSec   = (startSec + gOnDurationHours * 3600) % (24 * 3600);
  const int nowSec   = secondsSinceMidnight(lt);

  if (gOnDurationHours <= 0) return false;
  if (gOnDurationHours >= 24) return true;

  if (endSec > startSec) {
    return (nowSec >= startSec) && (nowSec < endSec);
  } else {
    return (nowSec >= startSec) || (nowSec < endSec);
  }
}

/* Return the next schedule boundary (start or end) as epoch time */
time_t nextScheduleBoundaryEpoch() {
  if (!timeIsValid()) return 0;

  tm lt;
  time_t now = time(nullptr);
  localtime_r(&now, &lt);

  const int startSec = gOnHour * 3600 + gOnMinute * 60 + gOnSecond;
  const int endSec   = (startSec + gOnDurationHours * 3600) % (24 * 3600);
  const int nowSec   = secondsSinceMidnight(lt);
  const bool onNow   = scheduleShouldBeOnNow();

  int targetSec = onNow ? endSec : startSec;

  tm ltNext = lt;
  ltNext.tm_hour = targetSec / 3600;
  ltNext.tm_min  = (targetSec % 3600) / 60;
  ltNext.tm_sec  = targetSec % 60;
  time_t candidate = mktime(&ltNext);

  if (candidate <= now) {
    ltNext.tm_mday += 1;
    candidate = mktime(&ltNext);
  }
  return candidate;
}

/* Build local ISO8601 with timezone offset (+/-hh:mm) */
String iso8601Now() {
  if (!timeIsValid()) return String("");
  time_t now = time(nullptr);

  tm lt;  // local time
  localtime_r(&now, &lt);

  tm gm;  // UTC time
  gmtime_r(&now, &gm);

  long offsetSec = mktime(&lt) - mktime(&gm); // local - UTC
  int sign = (offsetSec >= 0) ? +1 : -1;
  int offMinAbs = abs((int)(offsetSec / 60));
  int offH = offMinAbs / 60;
  int offM = offMinAbs % 60;

  char buf[40];
  snprintf(buf, sizeof(buf),
           "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d",
           lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
           lt.tm_hour, lt.tm_min, lt.tm_sec,
           (sign >= 0 ? '+' : '-'), offH, offM);
  return String(buf);
}

/* Publish status (retained) for single channel */
void publishStatus(int idx, bool force = false) {
  if (idx < 0 || idx >= RELAY_COUNT) return;

  static unsigned long lastPub[RELAY_COUNT] = {0};
  static bool lastState[RELAY_COUNT] = {false};
  static Mode lastMode[RELAY_COUNT] = {MODE_AUTO};

  bool changed = force || (lastState[idx] != relayState[idx]) || (lastMode[idx] != currentMode[idx]);
  unsigned long nowMs = millis();
  if (!changed && (nowMs - lastPub[idx]) < 60000UL) return; // heartbeat every 60s

  lastPub[idx] = nowMs;
  lastState[idx] = relayState[idx];
  lastMode[idx]  = currentMode[idx];

  StaticJsonDocument<384> doc;
  doc["system"]   = SYSTEM_ID;
  doc["deviceId"] = DEVICE_ID;
  doc["layer"]    = CHANNEL_LAYER[idx];
  doc["mode"]     = (currentMode[idx] == MODE_AUTO) ? "AUTO" : "MANUAL";
  doc["state"]    = relayState[idx];
  doc["outlet"]   = idx; // optional: which relay index

  JsonObject sched = doc.createNestedObject("schedule");
  sched["onHour"]        = gOnHour;
  sched["onMinute"]      = gOnMinute;
  sched["durationHours"] = gOnDurationHours;

  if (timeIsValid()) {
    time_t nextB = nextScheduleBoundaryEpoch();
    if (nextB > 0) doc["nextChangeEpoch"] = (uint32_t)nextB;
    doc["timestamp"] = iso8601Now();
  }

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  client.publish(TOPIC_STATUS, (const uint8_t*)buf, n, true /* retained */);
}

/* ---------- WiFi / NTP ---------- */
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed, will retry.");
  }
}

void syncTimeIfNeeded() {
  setenv("TZ", TZ_EUROPE_STOCKHOLM, 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  const uint32_t deadline = millis() + 15000;
  while (!timeIsValid() && millis() < deadline) {
    delay(250);
  }
  if (timeIsValid()) {
    Serial.printf("[Time] Synced: %s\n", iso8601Now().c_str());
  } else {
    Serial.println("[Time] NTP sync failed (timeout).");
  }
}

/* ---------- MQTT ---------- */
void mqttReconnect() {
  while (!client.connected()) {
    String cid = String("ESP32-LED-LAYERS-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(cid.c_str())) {
      client.subscribe(TOPIC_CMD);
      // publish initial status for all channels
      for (int i = 0; i < RELAY_COUNT; ++i) {
        publishStatus(i, true);
      }
    } else {
      delay(1500);
    }
  }
}

/* Handle incoming commands:

   1) Global schedule:
   {
     "system":"S01",
     "deviceId":"R01",
     "command":"SET_SCHEDULE",
     "onHour":5,
     "onMinute":0,
     "durationHours":16
   }

   2) Per-layer:
   {
     "system":"S01",
     "deviceId":"R01",
     "layer":"L01",
     "command":"ON|OFF|AUTO",
     "durationSec":<optional>
   }
*/
void handleCommand(JsonObjectConst root) {
  const char* sys = root["system"];
  const char* dev = root["deviceId"];
  const char* cmd = root["command"];

  if (!nonEmptyEq(SYSTEM_ID, sys) || !nonEmptyEq(DEVICE_ID, dev) || !cmd) {
    return;
  }

  String c = cmd;
  c.toUpperCase();

  // ---- Global schedule config ----
  if (c == "SET_SCHEDULE") {
    int onHour        = root["onHour"]        | gOnHour;
    int onMinute      = root["onMinute"]      | gOnMinute;
    int durationHours = root["durationHours"] | gOnDurationHours;

    // Clamp values to safe ranges
    if (onHour < 0)   onHour = 0;
    if (onHour > 23)  onHour = 23;
    if (onMinute < 0) onMinute = 0;
    if (onMinute > 59) onMinute = 59;
    if (durationHours < 0)  durationHours = 0;
    if (durationHours > 24) durationHours = 24;

    gOnHour          = onHour;
    gOnMinute        = onMinute;
    gOnSecond        = 0; // keep seconds at zero
    gOnDurationHours = durationHours;

    Serial.printf("[CMD] New schedule -> %02d:%02d for %d h\n",
                  gOnHour, gOnMinute, gOnDurationHours);

    // Force re-evaluation for AUTO channels
    for (int i = 0; i < RELAY_COUNT; ++i) {
      if (currentMode[i] == MODE_AUTO) {
        hasDriven[i] = false;   // so loop() will drive based on new schedule
        publishStatus(i, true); // immediately publish updated schedule/state
      }
    }
    return;
  }

  // ---- Per-layer commands (AUTO / ON / OFF) ----
  const char* layer = root["layer"];
  if (!layer) {
    return;
  }

  int idx = channelIndexByLayer(layer);
  if (idx < 0) {
    Serial.println("[CMD] No matching channel for given layer");
    return;
  }

  if (c == "AUTO") {
    currentMode[idx] = MODE_AUTO;
    manualUntil[idx] = 0; // clear override
    Serial.printf("[CMD] AUTO -> L=%s\n", CHANNEL_LAYER[idx]);
    publishStatus(idx, true);
    return;
  }

  if (c == "ON" || c == "OFF") {
    bool desired = (c == "ON");
    currentMode[idx] = MODE_MANUAL;
    manualUntil[idx] = 0;

    if (root.containsKey("durationSec") && root["durationSec"].is<uint32_t>() && timeIsValid()) {
      uint32_t dur = root["durationSec"].as<uint32_t>();
      manualUntil[idx] = time(nullptr) + dur;
    }

    if (!hasDriven[idx] || relayState[idx] != desired) {
      driveRelay(idx, desired);
      publishStatus(idx, true);
    }
    Serial.printf("[CMD] MANUAL -> L=%s: %s, until: %u\n",
                  CHANNEL_LAYER[idx],
                  desired ? "ON" : "OFF",
                  (unsigned)manualUntil[idx]);
  }
}

/* MQTT callback */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[MQTT] JSON error: %s\n", err.c_str());
    return;
  }
  JsonObjectConst root = doc.as<JsonObjectConst>();
  handleCommand(root);
}

/* ---------- Arduino ---------- */
void setup() {
  Serial.begin(115200);
  delay(100);

  for (int i = 0; i < RELAY_COUNT; ++i) {
    pinMode(RELAY_PINS[i], OUTPUT);
    relayState[i] = false;
    hasDriven[i]  = false;
    currentMode[i]= MODE_AUTO;
    manualUntil[i]= 0;
    driveRelay(i, false); // default OFF
  }

  connectWifi();
  syncTimeIfNeeded();

  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(20);
  client.setBufferSize(768);
  client.setCallback(mqttCallback);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWifi();
  if (!client.connected()) mqttReconnect();
  client.loop();

  time_t now = time(nullptr);

  for (int i = 0; i < RELAY_COUNT; ++i) {
    // Manual expiry per channel
    if (currentMode[i] == MODE_MANUAL && manualUntil[i] > 0 && timeIsValid()) {
      if (now >= manualUntil[i]) {
        currentMode[i] = MODE_AUTO;
        manualUntil[i] = 0;
        Serial.printf("[MANUAL] Duration ended -> AUTO for L=%s\n",
                      CHANNEL_LAYER[i]);
        publishStatus(i, true);
      }
    }

    bool desired = relayState[i];
    if (currentMode[i] == MODE_AUTO) {
      desired = scheduleShouldBeOnNow();
    }

    if (!hasDriven[i] || desired != relayState[i]) {
      driveRelay(i, desired);
      publishStatus(i, true);
    } else {
      publishStatus(i, false);
    }
  }

  static uint32_t lastNtpTry = 0;
  if (!timeIsValid() && (millis() - lastNtpTry) > 10000) {
    lastNtpTry = millis();
    syncTimeIfNeeded();
  }

  delay(500);
}
