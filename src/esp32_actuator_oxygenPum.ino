#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* WiFi / MQTT */
const char* ssid = "hydroleaf";
const char* password = "Reza11Reza11";

const char* mqtt_server = "192.168.8.3";
const int mqtt_port = 1883;
const char* TOPIC_MAIN  = "actuator/oxygenPump";

/* Target filter (leave "" to accept any) */
const char* SYSTEM_ID   = "S01";
const char* LAYER       = "L01";
const char* DEVICE_ID   = "R01";
const char* TARGET_NAME = "airPump";

/* Relay config */
const int  RELAY_PIN          = 10;
const bool RELAY_ACTIVE_HIGH  = true;

/* Globals */
WiFiClient espClient;
PubSubClient client(espClient);

bool currentState = false;
bool hasState     = false;

/* Helpers */
// Compare with optional empties; case-insensitive
static bool nonEmptyEq(const char* want, const char* got) {
  if (!want || *want == '\0') return true;
  if (!got) return false;
  return String(got).equalsIgnoreCase(want);
}

// Set relay output
void setRelay(bool on) {
  int level = RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
  digitalWrite(RELAY_PIN, level);
  currentState = on;
  hasState = true;
}

/* WiFi / MQTT */
void setupWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(200);
}

void mqttReconnect() {
  while (!client.connected()) {
    String cid = String("ESP32-RELAY-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(cid.c_str())) client.subscribe(TOPIC_MAIN);
    else delay(1500);
    }
  }

/* MQTT callback */
void callback(char* topic, byte* payload, unsigned int length) {
  // Parse JSON directly from buffer (small doc capacity)
  StaticJsonDocument<384> doc; // enough for given schema
  if (deserializeJson(doc, payload, length)) return;

  JsonObjectConst root = doc.as<JsonObjectConst>();
  if (root.isNull()) return;

  // Filter
  const char* sys = root["system"].as<const char*>();
  const char* lay = root["layer"].as<const char*>();
  const char* dev = root["deviceId"].as<const char*>();
  if (!nonEmptyEq(SYSTEM_ID, sys) || !nonEmptyEq(LAYER, lay) || !nonEmptyEq(DEVICE_ID, dev)) return;

  // Extract desired state for target controller
  bool desired;
  bool found = false;

  if (root["controllers"].is<JsonArrayConst>()) {
    for (JsonObjectConst c : root["controllers"].as<JsonArrayConst>()) {
      const char* name = c["name"].as<const char*>();
      // Only match by controller name; simpler and faster
      if (name && String(name).equalsIgnoreCase(TARGET_NAME)) {
        if (c["state"].is<bool>()) { desired = c["state"].as<bool>(); found = true; break; }
        if (c["state"].is<const char*>()) {
          String s = c["state"].as<const char*>(); s.toLowerCase();
          if (s=="on"||s=="true"||s=="1")  { desired = true;  found = true; break; }
          if (s=="off"||s=="false"||s=="0"){ desired = false; found = true; break; }
        }
  }
  }
  }
  if (!found) return;

  if (!hasState || desired != currentState) setRelay(desired);
}

/* Setup / Loop */
void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  setupWifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(20);
  client.setBufferSize(512); // smaller than 1024 is enough for ~200B payload
  client.setCallback(callback);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) setupWifi();
  if (!client.connected()) mqttReconnect();
  client.loop();
}
