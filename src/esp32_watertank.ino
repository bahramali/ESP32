#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>   // for EZO pH (UART1)

// ─────────── WiFi / MQTT ───────────
const char* ssid       = "hydroleaf";
const char* password   = "Reza11Reza11";

const char* mqtt_server = "192.168.8.3";
const int   mqtt_port   = 1883;

const char* sensor_topic = "waterTank";            // telemetry
const char* pump_topic   = "actuator/oxygenPump";

// ─────────── Device metadata ───────────
const char* DEVICE_ID      = "T01";
const char* LAYER          = "L01";
const char* SYSTEM_ID      = "S01";
const char* PUMP_DEVICE_ID = "R01";

// ─────────── Pins / Calibration ───────────
const int TDS_PIN  = 32;
const int TEMP_PIN = 4;    // DS18B20
const int DO_PIN   = 34;

// EZO pH (UART1 on ESP32)
HardwareSerial EZOph(1);
const int PH_RX_PIN = 16;  // ESP32 RX  <- EZO TX
const int PH_TX_PIN = 17;  // ESP32 TX  -> EZO RX
const uint32_t PH_BAUD = 9600;

// DO raw calibration
const int DO_RAW_SATURATION = 850 ;
const int DO_RAW_ZIRO = 20 ;

// ─────────── Behavior ───────────
const float DO_ON_THRESHOLD  = 3.5f;  // mg/L
const float DO_OFF_THRESHOLD = 4.0f;  // mg/L
const bool  FAILSAFE_ON_WHEN_SENSOR_OFFLINE = true;

const unsigned long PUMP_HEARTBEAT_MS = 5UL * 60UL * 1000UL;

// ─────────── Globals ───────────
WiFiClient espClient;
PubSubClient client(espClient);

OneWire oneWire(TEMP_PIN);
DallasTemperature ds18b20(&oneWire);

// diagnostics
unsigned long failedConnections   = 0;
unsigned long failedSends         = 0;
unsigned long disconnectedCounter = 0;

// pump mem
bool  previousPumpState   = false;
bool  hasPrevPumpState    = false;
unsigned long lastPumpBeatMs = 0;
String clientId;

// pH streaming state
float phValue = 0.0f;
bool  phOnline = false;
String _phLine;                  // UART line buffer

// ─────────── Helpers ───────────
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
  Serial.print("Syncing NTP");
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

String buildCompositeId(const char* systemId, const char* layer, const char* deviceId) {
  String s(systemId); s += "-"; s += layer; s += "-"; s += deviceId;
  return s;
}

void mqtt_reconnect() {
  if (clientId.isEmpty()) {
    clientId = String("ESP32-") + DEVICE_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  }
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(clientId.c_str())) {
      Serial.println("connected.");
    } else {
      Serial.print("MQTT failed, rc="); Serial.print(client.state());
      Serial.println(" retrying in 2s");
      failedConnections++;
      delay(2000);
    }
  }
}

// DO saturation curve (mg/L) vs temperature (°C)
float getDOSaturation(float tempC) {
  return (14.652f - 0.41022f * tempC + 0.007991f * tempC * tempC - 0.000077774f * tempC * tempC * tempC);
}

// Publish pump status (retained)
void publishPumpStatus(bool state, const String& timestamp, bool forceSend) {
  if (!forceSend && hasPrevPumpState && state == previousPumpState) return;

  StaticJsonDocument<384> doc;
  doc["system"]      = SYSTEM_ID;
  doc["layer"]       = LAYER;
  doc["deviceId"]    = PUMP_DEVICE_ID;
  doc["compositeId"] = buildCompositeId(SYSTEM_ID, LAYER, PUMP_DEVICE_ID);
  doc["timestamp"]   = timestamp;

  JsonArray controllers = doc.createNestedArray("controllers");
  JsonObject pump       = controllers.createNestedObject();
  pump["name"]  = "airPump";
  pump["type"]  = "relay";
  pump["state"]     = state;        // boolean
  pump["valueType"]  = "boolean";

  char msg[384];
    size_t n = serializeJson(doc, msg, sizeof(msg));

  bool ok = client.publish(pump_topic, (uint8_t*)msg, n, true);
  if (ok) {
    Serial.printf("Pump status publish: %s\n", state ? "true" : "false");
    previousPumpState = state;
    hasPrevPumpState  = true;
  } else {
    Serial.println("Pump status publish FAILED");
    failedSends++;
  }
}

// ─────────── EZO pH helpers (UART streaming) ───────────
void ezo_send(const char* cmd) {
  EZOph.write((const uint8_t*)cmd, strlen(cmd));
  EZOph.write('\r');
}

void ph_startContinuous() {
  ezo_send("C,1");   // enable continuous mode
  delay(50);
  ezo_send("R");     // start streaming
}

// Non-blocking poll; updates phValue/phOnline when CR received
void ph_poll() {
    while (EZOph.available()) {
      char c = EZOph.read();
    if (c == '\r') {            // end of line
      _phLine.trim();
      if (_phLine.length() && _phLine.charAt(0) != '?') {
        float v = _phLine.toFloat();
        if (!isnan(v) && v >= 0.0f && v <= 14.5f) {
          phValue  = v;
          phOnline = true;
        } else {
          phOnline = false;
        }
      } else {
        phOnline = false;
      }
      _phLine = "";
      } else if (c != '\n' && c != 0) {
      _phLine += c;
    }
}
}

// ─────────── Setup ───────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  analogReadResolution(12);
  ds18b20.begin();

  // EZO pH init on UART1
  EZOph.begin(PH_BAUD, SERIAL_8N1, PH_RX_PIN, PH_TX_PIN);
  ph_startContinuous();   // continuous streaming

  Serial.print("Reset reason: "); Serial.println(esp_reset_reason());

  setup_wifi();
  setup_time();

  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(30);
}

// ─────────── Loop ───────────
void loop() {
  // keep WiFi/MQTT alive
  if (WiFi.status() != WL_CONNECTED) {
    disconnectedCounter++;
    Serial.printf("WiFi disconnected (%lu). Reconnecting...\n", disconnectedCounter);
    setup_wifi();
  }
  if (!client.connected()) mqtt_reconnect();
  client.loop();

  // update streaming pH
  ph_poll();

  const String timestamp = getTimeISO8601();

  // ---- Read sensors ----
  ds18b20.requestTemperatures();
  float tempC = ds18b20.getTempCByIndex(0);
  bool tempOnline = (tempC > -10.0f && tempC < 60.0f);
  if (!tempOnline) tempC = 0.0f;

  // TDS (analog -> volts -> ppm)
  int   tdsRaw     = analogRead(TDS_PIN);
  float tdsVolt    = tdsRaw * (3.3f / 4095.0f);
  float tdsValue   = (133.42f * pow(tdsVolt, 3) - 255.86f * pow(tdsVolt, 2) + 857.39f * tdsVolt) * 0.5f;
  bool  tdsOnline  = (tdsVolt > 0.1f && tdsVolt < 3.0f && tdsValue > 0.0f && tdsValue < 2000.0f);
  if (!tdsOnline) tdsValue = 0.0f;

  // EC derived (mS/cm)
  float ecValue = tdsValue / 640.0f;

  // Dissolved Oxygen (two-point calib)
  int   doRaw    = analogRead(DO_PIN);
  bool  doOnline = (doRaw > 40 && doRaw < 1400);
  if (!doOnline) doRaw = 0;

  float do_mgL = 0.0f;
  if (tempOnline && doOnline) {
    const float do_sat = getDOSaturation(tempC);
    const float v0    = (float)DO_RAW_ZIRO;
    const float v100  = (float)DO_RAW_SATURATION;
    const float denom = v100 - v0;

    if (denom > 1e-3f) {
      float ratio = ((float)doRaw - v0) / denom;
      if (ratio < 0.0f) ratio = 0.0f;
      do_mgL = ratio * do_sat;
    } else {
      do_mgL = NAN;
    }
  }

  Serial.printf("(Temp=%.2f on=%d) (TDS=%.1fppm on=%d, EC=%.3fmS/cm) (DOraw=%d on=%d -> %.2f mg/L) (pH=%.3f on=%d)\n",
                tempC, tempOnline, tdsValue, tdsOnline, ecValue, doRaw, doOnline, do_mgL, phValue, phOnline);

  // ---- Pump logic ----
  bool decided = false;
  bool desired = false;

  if (tempOnline && doOnline) {
    if (do_mgL < DO_ON_THRESHOLD)       { desired = true;  decided = true; }
    else if (do_mgL > DO_OFF_THRESHOLD) { desired = false; decided = true; }
  } else if (FAILSAFE_ON_WHEN_SENSOR_OFFLINE) {
    desired = true; decided = true;
  }
  if (decided) publishPumpStatus(desired, timestamp, false);

  // heartbeat
  if (millis() - lastPumpBeatMs > PUMP_HEARTBEAT_MS) {
    bool hb = hasPrevPumpState ? previousPumpState : false;
    publishPumpStatus(hb, timestamp, true);
    lastPumpBeatMs = millis();
  }

  // ---- Build telemetry JSON ----
  StaticJsonDocument<1200> doc;
  doc["system"]    = SYSTEM_ID;
  doc["layer"]       = LAYER;
  doc["deviceId"]  = DEVICE_ID;

  doc["compositeId"] = buildCompositeId(SYSTEM_ID, LAYER, DEVICE_ID);
  doc["timestamp"] = timestamp;

  JsonArray sensorsArr = doc.createNestedArray("sensors");

  JsonObject tds = sensorsArr.createNestedObject();
  tds["sensorName"] = "HailegeTDS";
  tds["sensorType"]  = "dissolvedTDS";
  tds["value"]      = tdsValue;
  tds["unit"]       = "ppm";

  JsonObject ec = sensorsArr.createNestedObject();
  ec["sensorName"] = "HailegeTDS";
  ec["sensorType"]  = "dissolvedEC";
  ec["value"]      = ecValue;
  ec["unit"]       = "mS/cm";
  ec["source"]     = "calculated";

  JsonObject temp = sensorsArr.createNestedObject();
  temp["sensorName"] = "DS18B20";
  temp["sensorType"]  = "dissolvedTemp";
  temp["value"]      = tempC;
  temp["unit"]       = "°C";

  JsonObject dox = sensorsArr.createNestedObject();
  dox["sensorName"] = "DFROBOT";
  dox["sensorType"]  = "dissolvedOxygen";
  dox["value"]      = do_mgL;
  dox["unit"]       = "mg/L";

  JsonObject ph = sensorsArr.createNestedObject();
  ph["sensorName"] = "AtlasEZO_pH";
  ph["sensorType"] = "pH";
  ph["value"]      = phValue;     // latest streamed value
  ph["unit"]       = "pH";

  JsonObject health = doc.createNestedObject("health");
  health["HailegeTDS"] = tdsOnline;
  health["DS18B20"]    = tempOnline;
  health["DFROBOT"]    = doOnline;
  health["AtlasEZO_pH"]= phOnline;

  char out[1200];
  size_t n = serializeJson(doc, out, sizeof(out));

  bool ok = client.publish(sensor_topic, (uint8_t*)out, n, false);
  if (!ok) {
    Serial.println("MQTT sensor payload FAILED");
    failedSends++;
  }

  delay(1000);
}
