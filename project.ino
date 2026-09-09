// Distance traffic-light indicator + boom gate - ESP32 + 3x HC-SR04 +
// 1x RGB LED + 1x BMP180 pressure plate + 1x SG90 boom gate servo
//
// Per-sensor rule (same for top/left/right): <= 40cm = "blocked", > 40cm = "clear".
//
// Combined LED rule: green by default. When ALL THREE sensors are blocked at
// once, it goes red - and if that blocked state holds continuously for
// OVERSTAY_MS (10s), it escalates to yellow + the buzzer sounds. The moment
// any sensor clears again, it drops straight back to green and the buzzer
// turns off immediately.
//
// Gate rule: a vehicle pressing the plate opens the gate. Once the plate is
// released (vehicle has moved past it), the gate holds open for GATE_HOLD_MS
// then closes. This is independent of the LED logic above.

#include <Wire.h>
#include <ESP32Servo.h>

// GPIO pins - must match diagram.json's wiring
const uint8_t TOP_TRIG_PIN   = 33;
const uint8_t TOP_ECHO_PIN   = 5;
const uint8_t LEFT_TRIG_PIN  = 25;
const uint8_t LEFT_ECHO_PIN  = 26;
const uint8_t RIGHT_TRIG_PIN = 27;
const uint8_t RIGHT_ECHO_PIN = 32;

const uint8_t LED_R = 12;
const uint8_t LED_G = 13;
const uint8_t LED_B = 14;

const uint8_t PLATE_SDA_PIN = 21;
const uint8_t PLATE_SCL_PIN = 22;
const uint8_t BMP180_ADDR   = 0x77;

const uint8_t GATE_SERVO_PIN = 18;

const uint8_t  BUZZER_PIN      = 19;
const uint16_t BUZZER_FREQ_HZ  = 2000; // audible alarm pitch
bool buzzerOn = false; // tracks current state so we only log on a change, not every cycle

const float NEAR_CM = 40.0; // at/below this distance a sensor counts as "blocked"

const uint32_t READ_INTERVAL_MS = 150;
uint32_t lastReadAt = 0;

// --- overstay tuning ---
const uint32_t OVERSTAY_MS = 10000; // how long all-blocked must hold before it's an "overstay"
uint32_t blockedSince = 0;          // millis() when all-blocked started; 0 = not currently blocked

// --- pressure plate tuning ---
const int16_t  PRESSURE_DELTA_THRESHOLD = 120; // raw ADC counts above baseline = "pressed"
const uint32_t PLATE_DEBOUNCE_MS        = 300; // must be stable this long before it counts
int16_t plateBaseline = 0;
bool plateCandidateActive = false;
uint32_t plateCandidateSince = 0;
bool plateActive = false;

// --- gate FSM ---
const int      GATE_CLOSED_ANGLE = 0;
const int      GATE_OPEN_ANGLE   = 90;
const uint32_t GATE_HOLD_MS      = 3000; // how long the gate stays open once the plate clears
Servo gateServo;
bool gateOpen = false;
uint32_t gateHoldUntil = 0; // 0 = not yet counting down

// Fires one HC-SR04 (given its own TRIG/ECHO pins) and returns distance in cm,
// or -1 if nothing echoed back within range. Same math as before: sound takes
// ~58us per cm of round trip.
float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t durationUs = pulseIn(echoPin, HIGH, 25000UL); // ~430cm max range
  if (durationUs == 0) return -1;
  return durationUs / 58.0;
}

// A sensor only counts as "blocked" if it actually got a valid echo AND that
// echo is within NEAR_CM. A timeout (-1, nothing in range) is never blocked.
bool isBlocked(float distanceCm) {
  return distanceCm >= 0 && distanceCm <= NEAR_CM;
}

// Drives the 3 LED legs directly - true = that color's GPIO pin goes HIGH.
void setColor(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

// A plain digitalWrite(HIGH) holds the buzzer's diaphragm steady - silent,
// since it's simulated as a bare piezo speaker rather than a self-oscillating
// "active" buzzer chip. tone()/noTone() actually drives it at an audible
// frequency, which is what makes it produce sound in the simulator.
void setBuzzer(bool on) {
  if (on == buzzerOn) return; // only act (and log) on an actual change
  buzzerOn = on;
  if (on) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    Serial.println("[BUZZER] ON");
  } else {
    noTone(BUZZER_PIN);
    Serial.println("[BUZZER] OFF");
  }
}

// --- BMP180 minimal raw-pressure driver ---
// We only need a relative change to detect "something is pressing the
// plate", not a calibrated absolute pressure, so this skips the full
// temperature-compensation sequence and just reads the raw pressure
// register directly.
int16_t readRawPressure() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34); // start pressure measurement, OSS = 0
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (int16_t)((msb << 8) | lsb);
}

// Updates plateActive with the same debounce pattern used elsewhere: a
// candidate state must hold steady for PLATE_DEBOUNCE_MS before it commits.
void servicePlate() {
  int16_t raw = readRawPressure();
  int16_t delta = raw - plateBaseline;
  bool rawActive = delta > PRESSURE_DELTA_THRESHOLD;

  if (rawActive != plateCandidateActive) {
    plateCandidateActive = rawActive;
    plateCandidateSince = millis();
  }
  if ((millis() - plateCandidateSince) >= PLATE_DEBOUNCE_MS) {
    plateActive = plateCandidateActive;
  }
}

// Opens the gate on a plate press; once the plate clears, holds open for
// GATE_HOLD_MS before closing again.
void serviceGate() {
  uint32_t now = millis();

  if (!gateOpen) {
    if (plateActive) {
      gateServo.write(GATE_OPEN_ANGLE);
      gateOpen = true;
      gateHoldUntil = 0;
      Serial.println("[GATE] Opening");
    }
    return;
  }

  // Gate is open: start the hold timer once the plate is released, then
  // close once that hold time has passed.
  if (!plateActive && gateHoldUntil == 0) {
    gateHoldUntil = now + GATE_HOLD_MS;
  }
  if (gateHoldUntil != 0 && now >= gateHoldUntil) {
    gateServo.write(GATE_CLOSED_ANGLE);
    gateOpen = false;
    gateHoldUntil = 0;
    Serial.println("[GATE] Closing");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TOP_TRIG_PIN, OUTPUT);
  pinMode(TOP_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(PLATE_SDA_PIN, PLATE_SCL_PIN);
  plateBaseline = readRawPressure(); // ambient reading, no vehicle on the plate yet

  gateServo.setPeriodHertz(50);
  gateServo.attach(GATE_SERVO_PIN, 500, 2500);
  gateServo.write(GATE_CLOSED_ANGLE);
}

void loop() {
  uint32_t now = millis();
  if (now - lastReadAt < READ_INTERVAL_MS) return;
  lastReadAt = now;

  servicePlate();
  serviceGate();

  float topDistance   = readDistanceCm(TOP_TRIG_PIN, TOP_ECHO_PIN);
  float leftDistance  = readDistanceCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float rightDistance = readDistanceCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  bool topBlocked   = isBlocked(topDistance);
  bool leftBlocked  = isBlocked(leftDistance);
  bool rightBlocked = isBlocked(rightDistance);
  bool allBlocked   = topBlocked && leftBlocked && rightBlocked;

  const char *ledLabel;
  if (allBlocked) {
    if (blockedSince == 0) blockedSince = now; // just became blocked - start the overstay clock

    bool overstay = (now - blockedSince) >= OVERSTAY_MS;
    if (overstay) {
      setColor(true, true, false);   // yellow
      setBuzzer(true);
      ledLabel = "YELLOW (overstay)";
    } else {
      setColor(true, false, false);  // red
      setBuzzer(false);
      ledLabel = "RED";
    }
  } else {
    blockedSince = 0; // reset - the moment anything clears, overstay is over
    setColor(false, true, false);    // green
    setBuzzer(false);
    ledLabel = "GREEN";
  }

  Serial.printf("Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED %s\n",
                topDistance,   topBlocked   ? "blocked" : "clear",
                leftDistance,  leftBlocked  ? "blocked" : "clear",
                rightDistance, rightBlocked ? "blocked" : "clear",
                ledLabel);
}
