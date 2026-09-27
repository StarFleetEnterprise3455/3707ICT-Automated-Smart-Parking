// Smart Parking System: ESP32, 6x HC-SR04, 2x BMP180 pressure plates, servo gate, LEDs, buzzers and Blynk.
// Three ultrasonic sensors per bay provide fused occupancy; persistent disagreement/invalid readings trigger sensor faults.
// Local automation remains authoritative if Wi-Fi/Blynk is unavailable.

#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <time.h>
#include "secrets.h"
#include "cloud_config.h"
#include <BlynkSimpleEsp32_SSL.h>

const uint8_t ECHO_SHARED_PIN = 36;
const uint8_t TOP_TRIG_PIN   = 33;
const uint8_t LEFT_TRIG_PIN  = 25;
const uint8_t RIGHT_TRIG_PIN = 27;
const uint8_t BAY2_TOP_TRIG_PIN   = 15;
const uint8_t BAY2_LEFT_TRIG_PIN  = 2;
const uint8_t BAY2_RIGHT_TRIG_PIN = 4;
const uint8_t LED_R = 12;
const uint8_t LED_G = 13;
const uint8_t LED_B = 14;
const uint8_t LED2_R = 17;
const uint8_t LED2_G = 23;
const uint8_t PLATE_SDA_PIN = 21;
const uint8_t PLATE_SCL_PIN = 22;
const uint8_t BMP180_ADDR   = 0x77;
const uint8_t PLATE2_SDA_PIN = 26;
const uint8_t PLATE2_SCL_PIN = 32;
TwoWire ExitWire = TwoWire(1);
const uint8_t GATE_SERVO_PIN = 18;
const uint8_t GATE_LED_PIN   = 16;
const uint32_t GATE_LED_FLASH_MS = 200;
bool gateLedState = false;
uint32_t gateLedLastToggle = 0;
const uint8_t BUZZER_PIN  = 19;
const uint8_t BUZZER2_PIN = 5;
const uint16_t BUZZER_FREQ_HZ = 2000;
bool buzzerOn  = false;
bool buzzer2On = false;
const float NEAR_CM = 40.0;
const uint32_t READ_INTERVAL_MS = 150;
uint32_t lastReadAt = 0;
const uint32_t OVERSTAY_MS = 10000;
uint32_t blockedSince = 0;
uint32_t bay2BlockedSince = 0;
bool bay1OverstayActive = false;
bool bay2OverstayActive = false;
const uint32_t OCCUPANCY_STABILITY_MS = 600;

// Occupancy changes are debounced; persistent sensor disagreement/invalid readings become faults.
const uint32_t SENSOR_ANOMALY_CONFIRM_MS = 5000;

// Per-bay state used for stable occupancy and sensor anomaly detection.
struct BaySensorState
{
  bool confirmedOccupied = false;
  bool candidateOccupied = false;
  bool candidateValid = false;
  uint32_t candidateSince = 0;
  uint32_t anomalySince = 0;
  bool sensorFault = false;
};
BaySensorState bay1SensorState;
BaySensorState bay2SensorState;
const int16_t PRESSURE_DELTA_THRESHOLD = 120;
const uint32_t PLATE_DEBOUNCE_MS = 300;
int16_t plateBaseline = 0;
bool plateCandidateActive = false;
uint32_t plateCandidateSince = 0;
bool plateActive = false;
int16_t plateLastDelta = 0;
int16_t plateExitBaseline = 0;
bool plateExitCandidateActive = false;
uint32_t plateExitCandidateSince = 0;
bool plateExitActive = false;
int16_t plateExitLastDelta = 0;
const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE = 90;
const uint32_t GATE_HOLD_MS = 6000;
Servo gateServo;
bool gateOpen = false;
bool gateForExit = false;
uint32_t gateHoldUntil = 0;
bool bothBaysFull = false;
bool entryRefusedLatched = false;
float readDistanceCm(uint8_t trigPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  uint32_t durationUs =
      pulseIn(
          ECHO_SHARED_PIN,
          HIGH,
          25000UL);
  if (durationUs == 0)
  {
    return -1;
  }
  return durationUs / 58.0;
}
bool isBlocked(float distanceCm)
{
  return
      distanceCm >= 0 &&
      distanceCm <= NEAR_CM;
}

// Keeps the last confirmed occupancy during inconsistent readings and raises a fault after 5 seconds.
bool processBaySensorState(
    const char *bayName,
    float distance1,
    float distance2,
    float distance3,
    BaySensorState &state)
{
  uint32_t now = millis();
  bool valid1 = distance1 >= 0.0f;
  bool valid2 = distance2 >= 0.0f;
  bool valid3 = distance3 >= 0.0f;
  bool anomalyNow = false;
  bool unanimous = false;
  bool candidateOccupied = state.confirmedOccupied;
  if (valid1 && valid2 && valid3)
  {
    bool blocked1 = isBlocked(distance1);
    bool blocked2 = isBlocked(distance2);
    bool blocked3 = isBlocked(distance3);
    unanimous =
        (blocked1 == blocked2) &&
        (blocked2 == blocked3);
    if (unanimous)
    {
      candidateOccupied = blocked1;
    }
    else
    {
      anomalyNow = true;
    }
  }
  else
  {
    anomalyNow = true;
  }
  if (unanimous)
  {
    if (
        !state.candidateValid ||
        state.candidateOccupied != candidateOccupied)
    {
      state.candidateOccupied = candidateOccupied;
      state.candidateSince = now;
      state.candidateValid = true;
    }
    else if (
        state.confirmedOccupied != state.candidateOccupied &&
        now - state.candidateSince >= OCCUPANCY_STABILITY_MS)
    {
      state.confirmedOccupied = state.candidateOccupied;
      Serial.printf(
          "[OCCUPANCY] %s confirmed %s\n",
          bayName,
          state.confirmedOccupied ? "OCCUPIED" : "AVAILABLE");
    }
  }
  else
  {
    state.candidateValid = false;
  }
  bool previousFault = state.sensorFault;
  if (anomalyNow)
  {
    if (state.anomalySince == 0)
    {
      state.anomalySince = now;
    }
    if (now - state.anomalySince >= SENSOR_ANOMALY_CONFIRM_MS)
    {
      state.sensorFault = true;
    }
  }
  else
  {
    state.anomalySince = 0;
    state.sensorFault = false;
  }
  if (previousFault != state.sensorFault)
  {
    if (state.sensorFault)
    {
      Serial.printf(
          "[FAULT] %s sensor anomaly detected\n",
          bayName);
    }
    else
    {
      Serial.printf(
          "[FAULT] %s sensors recovered\n",
          bayName);
    }
  }
  return state.confirmedOccupied;
}
void setColor(bool r, bool g, bool b)
{
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}
void setColor2(bool r, bool g)
{
  digitalWrite(LED2_R, r);
  digitalWrite(LED2_G, g);
}
void setBuzzer(bool on)
{
  if (on == buzzerOn)
  {
    return;
  }
  buzzerOn = on;
  if (on)
  {
    tone(
        BUZZER_PIN,
        BUZZER_FREQ_HZ);
    Serial.println(
        "[BUZZER1] ON");
  }
  else
  {
    noTone(
        BUZZER_PIN);
    Serial.println(
        "[BUZZER1] OFF");
  }
}
void setBuzzer2(bool on)
{
  if (on == buzzer2On)
  {
    return;
  }
  buzzer2On = on;
  if (on)
  {
    tone(
        BUZZER2_PIN,
        BUZZER_FREQ_HZ);
    Serial.println(
        "[BUZZER2] ON");
  }
  else
  {
    noTone(
        BUZZER2_PIN);
    Serial.println(
        "[BUZZER2] OFF");
  }
}
int16_t readRawPressure(TwoWire &bus)
{
  bus.beginTransmission(
      BMP180_ADDR);
  bus.write(
      0xF4);
  bus.write(
      0x34);
  bus.endTransmission();
  delay(5);
  bus.beginTransmission(
      BMP180_ADDR);
  bus.write(
      0xF6);
  bus.endTransmission(
      false);
  bus.requestFrom(
      BMP180_ADDR,
      (uint8_t)2);
  if (bus.available() < 2)
  {
    return 0;
  }
  uint8_t msb =
      bus.read();
  uint8_t lsb =
      bus.read();
  return
      (int16_t)(
          (msb << 8) |
          lsb);
}
void servicePlate()
{
  int16_t raw =
      readRawPressure(
          Wire);
  int16_t delta =
      raw -
      plateBaseline;
  plateLastDelta =
      delta;
  bool rawActive =
      delta >
      PRESSURE_DELTA_THRESHOLD;
  if (
      rawActive !=
      plateCandidateActive)
  {
    plateCandidateActive =
        rawActive;
    plateCandidateSince =
        millis();
  }
  if (
      millis() -
          plateCandidateSince >=
      PLATE_DEBOUNCE_MS)
  {
    plateActive =
        plateCandidateActive;
  }
}
void servicePlateExit()
{
  int16_t raw =
      readRawPressure(
          ExitWire);
  int16_t delta =
      raw -
      plateExitBaseline;
  plateExitLastDelta =
      delta;
  bool rawActive =
      delta >
      PRESSURE_DELTA_THRESHOLD;
  if (
      rawActive !=
      plateExitCandidateActive)
  {
    plateExitCandidateActive =
        rawActive;
    plateExitCandidateSince =
        millis();
  }
  if (
      millis() -
          plateExitCandidateSince >=
      PLATE_DEBOUNCE_MS)
  {
    plateExitActive =
        plateExitCandidateActive;
  }
}

// Gate logic: entry is blocked when full; exit is always allowed; close delay remains 6 seconds.
void serviceGate()
{
  uint32_t now =
      millis();
  if (!gateOpen)
  {
    if (plateActive)
    {
      if (!bothBaysFull)
      {
        gateServo.write(
            GATE_OPEN_ANGLE);
        gateOpen =
            true;
        gateForExit =
            false;
        gateHoldUntil =
            0;
        Serial.println(
            "[GATE] Opening (entry)");
      }
      else if (!entryRefusedLatched)
      {
        entryRefusedLatched =
            true;
        Serial.println(
            "[GATE] Entry refused - capacity FULL");
      }
    }
    else
    {
      entryRefusedLatched =
          false;
      if (plateExitActive)
      {
        gateServo.write(
            GATE_OPEN_ANGLE);
        gateOpen =
            true;
        gateForExit =
            true;
        gateHoldUntil =
            0;
        Serial.println(
            "[GATE] Opening (exit)");
      }
    }
    return;
  }
  bool relevantActive =
      gateForExit
          ? plateExitActive
          : plateActive;
  if (
      !relevantActive &&
      gateHoldUntil == 0)
  {
    gateHoldUntil =
        now +
        GATE_HOLD_MS;
  }
  if (
      gateHoldUntil != 0 &&
      now >= gateHoldUntil)
  {
    gateServo.write(
        GATE_CLOSED_ANGLE);
    gateOpen =
        false;
    gateHoldUntil =
        0;
    Serial.println(
        "[GATE] Closing");
  }
}
void serviceGateLed()
{
  if (bothBaysFull)
  {
    if (!gateLedState)
    {
      gateLedState =
          true;
      digitalWrite(
          GATE_LED_PIN,
          HIGH);
    }
    return;
  }
  if (!gateOpen)
  {
    if (gateLedState)
    {
      gateLedState =
          false;
      digitalWrite(
          GATE_LED_PIN,
          LOW);
    }
    return;
  }
  uint32_t now =
      millis();
  if (
      now -
          gateLedLastToggle >=
      GATE_LED_FLASH_MS)
  {
    gateLedLastToggle =
        now;
    gateLedState =
        !gateLedState;
    digitalWrite(
        GATE_LED_PIN,
        gateLedState);
  }
}

// Blynk telemetry contract: V0-V7 via MQTT/TLS, V8-V15 via HTTPS/TLS.
struct ParkingTelemetry
{
  bool bay1Occupied = false;       // V0
  bool bay2Occupied = false;       // V1
  int availableSpaces = 2;         // V2
  int gateState = 0;               // V3
  uint32_t bay1DurationSec = 0;     // V4
  uint32_t bay2DurationSec = 0;     // V5
  bool entryPlate = false;          // V6
  bool exitPlate = false;           // V7
  bool fullCapacity = false;        // V8
  bool bay1Overstay = false;        // V9
  bool bay2Overstay = false;        // V10
  bool sensorFault = false;         // V11
  int wifiRSSI = -120;              // V12
  String systemStatus = "OFFLINE";  // V13
  String alertMessage = "NONE";     // V14
  int wifiQuality = 0;              // V15
};
ParkingTelemetry telemetry;
struct MqttEventState
{
  bool bay1Occupied = false;
  bool bay2Occupied = false;
  int availableSpaces = 2;
  int gateState = 0;
  bool entryPlate = false;
  bool exitPlate = false;
};
struct HttpsEventState
{
  bool fullCapacity = false;
  bool bay1Overstay = false;
  bool bay2Overstay = false;
  bool sensorFault = false;
  String systemStatus =
      "OFFLINE";
  String alertMessage =
      "NONE";
};
MqttEventState lastMqttState;
HttpsEventState lastHttpsState;
bool mqttStateValid = false;
bool httpsStateValid = false;
WiFiClientSecure mqttTlsClient;
WiFiClientSecure httpsTlsClient;
PubSubClient mqttClient(
    mqttTlsClient);
bool wifiWasConnected =
    false;
bool timeSyncStarted =
    false;
bool timeIsValid =
    false;
uint32_t lastWiFiRetry =
    0;
uint32_t lastMQTTRetry =
    0;
uint32_t lastCloudPublish =
    0;
const uint32_t WIFI_RETRY_INTERVAL_MS =
    10000;
const uint32_t MQTT_RETRY_INTERVAL_MS =
    2000;
const uint32_t CLOUD_HEARTBEAT_INTERVAL_MS =
    30000;
String urlEncode(
    const String &value)
{
  String encoded =
      "";
  const char hex[] =
      "0123456789ABCDEF";
  for (
      size_t i = 0;
      i < value.length();
      ++i)
  {
    unsigned char c =
        static_cast<unsigned char>(
            value.charAt(i));
    if (
        (c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '-' ||
        c == '_' ||
        c == '.' ||
        c == '~')
    {
      encoded +=
          static_cast<char>(c);
    }
    else
    {
      encoded +=
          '%';
      encoded +=
          hex[
              (c >> 4) &
              0x0F];
      encoded +=
          hex[
              c &
              0x0F];
    }
  }
  return encoded;
}

// Convert RSSI to an intuitive 0-100% dashboard quality value.
int calculateWifiQuality(
    int rssi)
{
  if (rssi <= -100)
  {
    return 0;
  }
  if (rssi >= -50)
  {
    return 100;
  }
  return
      2 *
      (rssi + 100);
}
void startWiFi()
{
  Serial.println(
      "[CLOUD] Starting Wi-Fi...");
  WiFi.mode(
      WIFI_STA);
  if (
      String(WIFI_SSID) ==
      "Wokwi-GUEST")
  {
    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD,
        6);
  }
  else
  {
    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);
  }
}
void maintainWiFi()
{
  if (
      WiFi.status() ==
      WL_CONNECTED)
  {
    telemetry.wifiRSSI =
        WiFi.RSSI();
    telemetry.wifiQuality =
        calculateWifiQuality(
            telemetry.wifiRSSI);
    if (!wifiWasConnected)
    {
      wifiWasConnected =
          true;
      telemetry.systemStatus =
          "DEGRADED";
      Serial.printf(
          "[CLOUD] Wi-Fi connected | IP %s | RSSI %d dBm | Quality %d%%\n",
          WiFi.localIP().toString().c_str(),
          telemetry.wifiRSSI,
          telemetry.wifiQuality);
    }
    return;
  }
  telemetry.wifiRSSI =
      -120;
  telemetry.wifiQuality =
      0;
  telemetry.systemStatus =
      "OFFLINE";
  if (wifiWasConnected)
  {
    wifiWasConnected =
        false;
    if (mqttClient.connected())
    {
      mqttClient.disconnect();
    }
    timeSyncStarted =
        false;
    timeIsValid =
        false;
    mqttStateValid =
        false;
    httpsStateValid =
        false;
    Serial.println(
        "[CLOUD] Wi-Fi lost. Local parking control continues.");
  }
  uint32_t now =
      millis();
  if (
      now -
          lastWiFiRetry >=
      WIFI_RETRY_INTERVAL_MS)
  {
    lastWiFiRetry =
        now;
    Serial.println(
        "[CLOUD] Retrying Wi-Fi...");
    WiFi.disconnect();
    if (
        String(WIFI_SSID) ==
        "Wokwi-GUEST")
    {
      WiFi.begin(
          WIFI_SSID,
          WIFI_PASSWORD,
          6);
    }
    else
    {
      WiFi.begin(
          WIFI_SSID,
          WIFI_PASSWORD);
    }
  }
}
void maintainTimeSync()
{
  if (
      WiFi.status() !=
      WL_CONNECTED)
  {
    return;
  }
  if (!timeSyncStarted)
  {
    configTime(
        0,
        0,
        "pool.ntp.org",
        "time.nist.gov");
    timeSyncStarted =
        true;
    Serial.println(
        "[CLOUD] NTP time sync started...");
  }
  if (timeIsValid)
  {
    return;
  }
  time_t now =
      time(nullptr);
  if (now > 1700000000)
  {
    timeIsValid =
        true;
    Serial.println(
        "[CLOUD] System time synchronised for TLS validation.");
  }
}
void mqttCallback(
    char *topic,
    byte *payload,
    unsigned int length)
{
  Serial.print(
      "[CLOUD] MQTT RX [");
  Serial.print(
      topic);
  Serial.print(
      "]: ");
  for (
      unsigned int i = 0;
      i < length;
      ++i)
  {
    Serial.print(
        (char)payload[i]);
  }
  Serial.println();
}
void publishDeviceInfo()
{
  if (!mqttClient.connected())
  {
    return;
  }
  JsonDocument doc;
  doc["tmpl"] =
      BLYNK_TEMPLATE_ID;
  doc["ver"] =
      FIRMWARE_VERSION;
  doc["build"] =
      String(__DATE__) +
      " " +
      String(__TIME__);
  doc["type"] =
      BLYNK_TEMPLATE_ID;
  doc["rxbuff"] =
      1024;
  char payload[256];
  if (
      serializeJson(
          doc,
          payload,
          sizeof(payload)) == 0)
  {
    return;
  }
  mqttClient.publish(
      "info/mcu",
      payload);
}

// Primary parking telemetry is published to Blynk with MQTT over TLS.
bool publishMqttTelemetry()
{
  if (!mqttClient.connected())
  {
    return false;
  }
  JsonDocument doc;
  doc["Bay1Occupied"] =
      telemetry.bay1Occupied
          ? 1
          : 0;
  doc["Bay2Occupied"] =
      telemetry.bay2Occupied
          ? 1
          : 0;
  doc["AvailableSpaces"] =
      telemetry.availableSpaces;
  doc["GateState"] =
      telemetry.gateState;
  doc["Bay1DurationSec"] =
      telemetry.bay1DurationSec;
  doc["Bay2DurationSec"] =
      telemetry.bay2DurationSec;
  doc["EntryPlate"] =
      telemetry.entryPlate
          ? 1
          : 0;
  doc["ExitPlate"] =
      telemetry.exitPlate
          ? 1
          : 0;
  char payload[512];
  if (
      serializeJson(
          doc,
          payload,
          sizeof(payload)) == 0)
  {
    return false;
  }
  bool ok =
      mqttClient.publish(
          "batch_ds",
          payload);
  if (!ok)
  {
    Serial.println(
        "[CLOUD] MQTT V0-V7 publish failed.");
  }
  return ok;
}

// Additional status/alert datastreams are published to Blynk with HTTPS over TLS.
bool publishHttpsTelemetry()
{
  if (
      WiFi.status() !=
          WL_CONNECTED ||
      !timeIsValid)
  {
    return false;
  }
  String url =
      "https://" +
      String(BLYNK_MQTT_HOST) +
      "/external/api/batch/update?token=" +
      urlEncode(
          String(
              BLYNK_AUTH_TOKEN));
  url +=
      "&v8=" +
      String(
          telemetry.fullCapacity
              ? 1
              : 0);
  url +=
      "&v9=" +
      String(
          telemetry.bay1Overstay
              ? 1
              : 0);
  url +=
      "&v10=" +
      String(
          telemetry.bay2Overstay
              ? 1
              : 0);
  url +=
      "&v11=" +
      String(
          telemetry.sensorFault
              ? 1
              : 0);
  url +=
      "&v12=" +
      String(
          telemetry.wifiRSSI);
  url +=
      "&v13=" +
      urlEncode(
          telemetry.systemStatus);
  url +=
      "&v14=" +
      urlEncode(
          telemetry.alertMessage);
  url +=
      "&v15=" +
      String(
          telemetry.wifiQuality);
  HTTPClient http;
  http.setTimeout(
      750);
  if (
      !http.begin(
          httpsTlsClient,
          url))
  {
    Serial.println(
        "[CLOUD] HTTPS setup failed.");
    return false;
  }
  int code =
      http.GET();
  bool ok =
      code >= 200 &&
      code < 300;
  if (!ok)
  {
    Serial.printf(
        "[CLOUD] HTTPS V8-V15 failed, code %d\n",
        code);
    String body =
        http.getString();
    if (body.length())
    {
      Serial.println(
          body);
    }
  }
  http.end();
  return ok;
}
bool mqttEventChanged()
{
  if (!mqttStateValid)
  {
    return true;
  }
  return
      telemetry.bay1Occupied !=
          lastMqttState.bay1Occupied ||
      telemetry.bay2Occupied !=
          lastMqttState.bay2Occupied ||
      telemetry.availableSpaces !=
          lastMqttState.availableSpaces ||
      telemetry.gateState !=
          lastMqttState.gateState ||
      telemetry.entryPlate !=
          lastMqttState.entryPlate ||
      telemetry.exitPlate !=
          lastMqttState.exitPlate;
}
bool httpsEventChanged()
{
  if (!httpsStateValid)
  {
    return true;
  }
  return
      telemetry.fullCapacity !=
          lastHttpsState.fullCapacity ||
      telemetry.bay1Overstay !=
          lastHttpsState.bay1Overstay ||
      telemetry.bay2Overstay !=
          lastHttpsState.bay2Overstay ||
      telemetry.sensorFault !=
          lastHttpsState.sensorFault ||
      telemetry.systemStatus !=
          lastHttpsState.systemStatus ||
      telemetry.alertMessage !=
          lastHttpsState.alertMessage;
}
void rememberMqttState()
{
  lastMqttState.bay1Occupied =
      telemetry.bay1Occupied;
  lastMqttState.bay2Occupied =
      telemetry.bay2Occupied;
  lastMqttState.availableSpaces =
      telemetry.availableSpaces;
  lastMqttState.gateState =
      telemetry.gateState;
  lastMqttState.entryPlate =
      telemetry.entryPlate;
  lastMqttState.exitPlate =
      telemetry.exitPlate;
  mqttStateValid =
      true;
}
void rememberHttpsState()
{
  lastHttpsState.fullCapacity =
      telemetry.fullCapacity;
  lastHttpsState.bay1Overstay =
      telemetry.bay1Overstay;
  lastHttpsState.bay2Overstay =
      telemetry.bay2Overstay;
  lastHttpsState.sensorFault =
      telemetry.sensorFault;
  lastHttpsState.systemStatus =
      telemetry.systemStatus;
  lastHttpsState.alertMessage =
      telemetry.alertMessage;
  httpsStateValid =
      true;
}
void publishCloudSnapshot()
{
  bool mqttOK =
      publishMqttTelemetry();
  if (mqttOK)
  {
    rememberMqttState();
  }
  if (mqttClient.connected())
  {
    mqttClient.loop();
  }
  bool httpsOK =
      publishHttpsTelemetry();
  if (httpsOK)
  {
    rememberHttpsState();
  }
  Serial.printf(
      "[CLOUD] Heartbeat -> MQTT:%s HTTPS:%s | RSSI:%d dBm | WiFi:%d%%\n",
      mqttOK
          ? "OK"
          : "FAIL",
      httpsOK
          ? "OK"
          : "FAIL",
      telemetry.wifiRSSI,
      telemetry.wifiQuality);
}
void setupCloudClients()
{
  mqttTlsClient.setCACert(
      BLYNK_DEFAULT_ROOT_CA);
  httpsTlsClient.setCACert(
      BLYNK_DEFAULT_ROOT_CA);
  mqttTlsClient.setHandshakeTimeout(
      10);
  httpsTlsClient.setHandshakeTimeout(
      10);
  mqttClient.setServer(
      BLYNK_MQTT_HOST,
      BLYNK_MQTT_PORT);
  mqttClient.setCallback(
      mqttCallback);
  mqttClient.setKeepAlive(
      45);
  mqttClient.setBufferSize(
      1024);
  mqttClient.setSocketTimeout(
      3);
  startWiFi();
}
void connectMQTT()
{
  if (
      WiFi.status() !=
          WL_CONNECTED ||
      !timeIsValid ||
      mqttClient.connected())
  {
    return;
  }
  uint32_t now =
      millis();
  if (
      lastMQTTRetry != 0 &&
      now -
              lastMQTTRetry <
          MQTT_RETRY_INTERVAL_MS)
  {
    return;
  }
  lastMQTTRetry =
      now;
  Serial.printf(
      "[CLOUD] Connecting MQTT -> %s:%u\n",
      BLYNK_MQTT_HOST,
      BLYNK_MQTT_PORT);
  bool connected =
      mqttClient.connect(
          "smart-parking-integrated",
          "device",
          BLYNK_AUTH_TOKEN);
  if (connected)
  {
    telemetry.systemStatus =
        "ONLINE";
    Serial.println(
        "[CLOUD] MQTT connected.");
    mqttClient.subscribe(
        "downlink/#");
    publishDeviceInfo();
    mqttStateValid =
        false;
    httpsStateValid =
        false;
    publishCloudSnapshot();
    lastCloudPublish =
        millis();
  }
  else
  {
    telemetry.systemStatus =
        "DEGRADED";
    Serial.printf(
        "[CLOUD] MQTT connect failed, state=%d. Local control continues.\n",
        mqttClient.state());
  }
}
void maintainMQTT()
{
  if (
      WiFi.status() !=
          WL_CONNECTED ||
      !timeIsValid)
  {
    return;
  }
  if (!mqttClient.connected())
  {
    telemetry.systemStatus =
        "DEGRADED";
    connectMQTT();
    return;
  }
  telemetry.systemStatus =
      "ONLINE";
  mqttClient.loop();
}
void publishChangedTelemetry()
{
  if (
      WiFi.status() !=
          WL_CONNECTED ||
      !timeIsValid)
  {
    return;
  }
  if (
      mqttClient.connected() &&
      mqttEventChanged())
  {
    if (publishMqttTelemetry())
    {
      rememberMqttState();
      Serial.println(
          "[CLOUD] Immediate MQTT state update sent.");
    }
  }
  if (httpsEventChanged())
  {
    if (publishHttpsTelemetry())
    {
      rememberHttpsState();
      Serial.println(
          "[CLOUD] Immediate HTTPS state update sent.");
    }
  }
}
void maintainCloudHeartbeat()
{
  if (
      WiFi.status() !=
          WL_CONNECTED ||
      !timeIsValid)
  {
    return;
  }
  uint32_t now =
      millis();
  if (
      now -
          lastCloudPublish >=
      CLOUD_HEARTBEAT_INTERVAL_MS)
  {
    lastCloudPublish =
        now;
    publishCloudSnapshot();
  }
}

// Export controller-owned state to cloud telemetry; cloud never controls safety-critical logic.
void syncTelemetryFromController(
    bool bay1Occupied,
    bool bay2Occupied,
    bool bay1Overstay,
    bool bay2Overstay)
{
  telemetry.bay1Occupied =
      bay1Occupied;
  telemetry.bay2Occupied =
      bay2Occupied;
  telemetry.availableSpaces =
      2 -
      (bay1Occupied ? 1 : 0) -
      (bay2Occupied ? 1 : 0);
  telemetry.fullCapacity =
      bothBaysFull;
  telemetry.entryPlate =
      plateActive;
  telemetry.exitPlate =
      plateExitActive;
  telemetry.bay1Overstay =
      bay1Overstay;
  telemetry.bay2Overstay =
      bay2Overstay;
  telemetry.bay1DurationSec =
      (
          bay1Occupied &&
          blockedSince != 0)
          ? (
                millis() -
                blockedSince) /
                1000UL
          : 0;
  telemetry.bay2DurationSec =
      (
          bay2Occupied &&
          bay2BlockedSince != 0)
          ? (
                millis() -
                bay2BlockedSince) /
                1000UL
          : 0;
  if (gateOpen)
  {
    if (gateHoldUntil != 0)
    {
      telemetry.gateState =
          3;
    }
    else
    {
      telemetry.gateState =
          gateForExit
              ? 2
              : 1;
    }
  }
  else if (bothBaysFull)
  {
    telemetry.gateState =
        4;
  }
  else
  {
    telemetry.gateState =
        0;
  }
  telemetry.sensorFault =
      bay1SensorState.sensorFault ||
      bay2SensorState.sensorFault;
  if (
      bay1SensorState.sensorFault &&
      bay2SensorState.sensorFault)
  {
    telemetry.alertMessage =
        "BOTH BAYS SENSOR FAULT";
  }
  else if (bay1SensorState.sensorFault)
  {
    telemetry.alertMessage =
        "BAY 1 SENSOR FAULT";
  }
  else if (bay2SensorState.sensorFault)
  {
    telemetry.alertMessage =
        "BAY 2 SENSOR FAULT";
  }
  else if (entryRefusedLatched)
  {
    telemetry.alertMessage =
        "ENTRY REFUSED - FULL";
  }
  else if (
      bay1Overstay &&
      bay2Overstay)
  {
    telemetry.alertMessage =
        "BOTH BAYS OVERSTAY";
  }
  else if (bay1Overstay)
  {
    telemetry.alertMessage =
        "BAY 1 OVERSTAY";
  }
  else if (bay2Overstay)
  {
    telemetry.alertMessage =
        "BAY 2 OVERSTAY";
  }
  else if (bothBaysFull)
  {
    telemetry.alertMessage =
        "CAR PARK FULL";
  }
  else
  {
    telemetry.alertMessage =
        "NONE";
  }
}

// Cloud servicing is best-effort so local parking automation continues during outages.
void serviceCloud()
{
  maintainWiFi();
  maintainTimeSync();
  maintainMQTT();
  publishChangedTelemetry();
  maintainCloudHeartbeat();
}

// Initialise local hardware first, then start the cloud subsystem.
void setup()
{
  Serial.begin(
      115200);
  pinMode(
      ECHO_SHARED_PIN,
      INPUT);
  pinMode(
      TOP_TRIG_PIN,
      OUTPUT);
  pinMode(
      LEFT_TRIG_PIN,
      OUTPUT);
  pinMode(
      RIGHT_TRIG_PIN,
      OUTPUT);
  pinMode(
      BAY2_TOP_TRIG_PIN,
      OUTPUT);
  pinMode(
      BAY2_LEFT_TRIG_PIN,
      OUTPUT);
  pinMode(
      BAY2_RIGHT_TRIG_PIN,
      OUTPUT);
  pinMode(
      LED_R,
      OUTPUT);
  pinMode(
      LED_G,
      OUTPUT);
  pinMode(
      LED_B,
      OUTPUT);
  pinMode(
      LED2_R,
      OUTPUT);
  pinMode(
      LED2_G,
      OUTPUT);
  pinMode(
      BUZZER_PIN,
      OUTPUT);
  pinMode(
      BUZZER2_PIN,
      OUTPUT);
  pinMode(
      GATE_LED_PIN,
      OUTPUT);
  Wire.begin(
      PLATE_SDA_PIN,
      PLATE_SCL_PIN);
  plateBaseline =
      readRawPressure(
          Wire);
  ExitWire.begin(
      PLATE2_SDA_PIN,
      PLATE2_SCL_PIN);
  plateExitBaseline =
      readRawPressure(
          ExitWire);
  gateServo.setPeriodHertz(
      50);
  gateServo.attach(
      GATE_SERVO_PIN,
      500,
      2500);
  gateServo.write(
      GATE_CLOSED_ANGLE);
  setupCloudClients();
  Serial.println(
      "[SYSTEM] Local parking controller started; cloud is best-effort.");
}

// Main loop prioritises local sensing/control, then services cloud communication.
void loop()
{
  uint32_t now =
      millis();
  if (
      now -
          lastReadAt >=
      READ_INTERVAL_MS)
  {
    lastReadAt =
        now;
    float topDistance =
        readDistanceCm(
            TOP_TRIG_PIN);
    float leftDistance =
        readDistanceCm(
            LEFT_TRIG_PIN);
    float rightDistance =
        readDistanceCm(
            RIGHT_TRIG_PIN);
    bool topBlocked =
        isBlocked(
            topDistance);
    bool leftBlocked =
        isBlocked(
            leftDistance);
    bool rightBlocked =
        isBlocked(
            rightDistance);
    bool allBlocked =
        processBaySensorState(
            "BAY 1",
            topDistance,
            leftDistance,
            rightDistance,
            bay1SensorState);
    const char *ledLabel;
    if (allBlocked)
    {
      if (blockedSince == 0)
      {
        blockedSince =
            now;
      }
      bay1OverstayActive =
          (
              now -
              blockedSince) >=
          OVERSTAY_MS;
      if (bay1OverstayActive)
      {
        setColor(
            true,
            true,
            false);
        ledLabel =
            "YELLOW (overstay)";
      }
      else
      {
        setColor(
            true,
            false,
            false);
        ledLabel =
            "RED";
      }
    }
    else
    {
      blockedSince =
          0;
      bay1OverstayActive =
          false;
      setColor(
          false,
          true,
          false);
      ledLabel =
          "GREEN";
    }
    Serial.printf(
        "Bay1 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED %s\n",
        topDistance,
        topBlocked
            ? "blocked"
            : "clear",
        leftDistance,
        leftBlocked
            ? "blocked"
            : "clear",
        rightDistance,
        rightBlocked
            ? "blocked"
            : "clear",
        ledLabel);
    float bay2TopDistance =
        readDistanceCm(
            BAY2_TOP_TRIG_PIN);
    float bay2LeftDistance =
        readDistanceCm(
            BAY2_LEFT_TRIG_PIN);
    float bay2RightDistance =
        readDistanceCm(
            BAY2_RIGHT_TRIG_PIN);
    bool bay2TopBlocked =
        isBlocked(
            bay2TopDistance);
    bool bay2LeftBlocked =
        isBlocked(
            bay2LeftDistance);
    bool bay2RightBlocked =
        isBlocked(
            bay2RightDistance);
    bool bay2AllBlocked =
        processBaySensorState(
            "BAY 2",
            bay2TopDistance,
            bay2LeftDistance,
            bay2RightDistance,
            bay2SensorState);
    const char *led2Label;
    if (bay2AllBlocked)
    {
      if (bay2BlockedSince == 0)
      {
        bay2BlockedSince =
            now;
      }
      bay2OverstayActive =
          (
              now -
              bay2BlockedSince) >=
          OVERSTAY_MS;
      if (bay2OverstayActive)
      {
        setColor2(
            true,
            true);
        led2Label =
            "YELLOW (overstay)";
      }
      else
      {
        setColor2(
            true,
            false);
        led2Label =
            "RED";
      }
    }
    else
    {
      bay2BlockedSince =
          0;
      bay2OverstayActive =
          false;
      setColor2(
          false,
          true);
      led2Label =
          "GREEN";
    }
    Serial.printf(
        "Bay2 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED2 %s\n",
        bay2TopDistance,
        bay2TopBlocked
            ? "blocked"
            : "clear",
        bay2LeftDistance,
        bay2LeftBlocked
            ? "blocked"
            : "clear",
        bay2RightDistance,
        bay2RightBlocked
            ? "blocked"
            : "clear",
        led2Label);
    bothBaysFull =
        allBlocked &&
        bay2AllBlocked;
    servicePlate();
    servicePlateExit();
    serviceGate();
    serviceGateLed();

    // Warn locally when an arriving vehicle is refused because both bays are occupied.
    bool fullCapacityEntryWarning =
        entryRefusedLatched;
    setBuzzer(
        bay1OverstayActive ||
        bay1SensorState.sensorFault ||
        fullCapacityEntryWarning);
    setBuzzer2(
        bay2OverstayActive ||
        bay2SensorState.sensorFault ||
        fullCapacityEntryWarning);
    Serial.printf(
        "Entry plate delta: %d / %d%s | Exit plate delta: %d / %d%s%s\n",
        plateLastDelta,
        PRESSURE_DELTA_THRESHOLD,
        plateActive
            ? " (PRESSED)"
            : "",
        plateExitLastDelta,
        PRESSURE_DELTA_THRESHOLD,
        plateExitActive
            ? " (PRESSED)"
            : "",
        bothBaysFull
            ? " | CAPACITY FULL"
            : "");
    syncTelemetryFromController(
        allBlocked,
        bay2AllBlocked,
        bay1OverstayActive,
        bay2OverstayActive);
  }
  serviceCloud();
}
