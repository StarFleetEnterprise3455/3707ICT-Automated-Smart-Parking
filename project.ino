// Distance traffic-light indicator + boom gate - ESP32 + 6x HC-SR04 (2 bays
// of 3) + 1x RGB LED + 1x BMP180 pressure plate + 1x SG90 boom gate servo
//
// Sensor layout per bay - 3 HC-SR04 all aimed into the SAME bay, from
// different positions, so a vehicle sitting in that bay blocks all three at
// once (this is what makes "all 3 blocked" a reasonable stand-in for "bay
// occupied", per the pitch's 3-sensor fusion idea):
//   TOP   - mounted at the back wall of the bay, facing forward into it.
//   LEFT  - mounted on the bay's left-hand boundary, facing across it.
//   RIGHT - mounted on the bay's right-hand boundary, facing across it.
// Bay 1 = sensorTop/sensorLeft/sensorRight. Bay 2 = bay2Top/bay2Left/bay2Right,
// same arrangement, duplicated for a second physical bay.
//
// All 6 sensors share ONE echo pin (ECHO_SHARED_PIN) to free up GPIOs for
// other hardware. This is safe only because every sensor is triggered and
// read one at a time, in sequence - never two at once - so their echo
// pulses never overlap on the shared line.
//
// Per-sensor rule (same for every sensor, both bays): <= 40cm = "blocked", > 40cm = "clear".
//
// LED rule (same for both bays, each with its own LED and its own overstay
// timer): green by default. When ALL THREE of that bay's sensors are blocked
// at once, it goes red - and if that holds continuously for OVERSTAY_MS
// (10s), it escalates to yellow. The moment any of the 3 clears, it drops
// straight back to green immediately.
// Each bay has its own buzzer, sounding only during that bay's own overstay
// (yellow) - Bay 1's buzzer never sounds for Bay 2's overstay and vice versa.
//
// Gate rule: two plates now - "plate" (entry) and "plateExit" (exit), each on
// its own I2C bus (BMP180 has a fixed address, so two can't share one bus).
// Pressing EITHER plate opens the gate. Once that same plate is released
// (vehicle has moved past it), the gate holds open for GATE_HOLD_MS (6s)
// then closes. This is independent of the bay logic above.
//
// gateLed flashes for the WHOLE time the gate is open - from the moment a
// plate press triggers it open, all the way through the post-clear hold,
// until the gate actually closes. Not just while a plate is being pressed.

#include <Wire.h>
#include <ESP32Servo.h>

// GPIO pins - must match diagram.json's wiring
const uint8_t ECHO_SHARED_PIN = 36; // VP - shared by all 6 sensors, see note above

// Bay 1
const uint8_t TOP_TRIG_PIN   = 33;
const uint8_t LEFT_TRIG_PIN  = 25;
const uint8_t RIGHT_TRIG_PIN = 27;

// Bay 2 (same top/left/right arrangement, duplicated)
const uint8_t BAY2_TOP_TRIG_PIN   = 15;
const uint8_t BAY2_LEFT_TRIG_PIN  = 2;
const uint8_t BAY2_RIGHT_TRIG_PIN = 4;

const uint8_t LED_R = 12;
const uint8_t LED_G = 13;
const uint8_t LED_B = 14;

// Bay 2's LED only has Red+Green wired (2 GPIOs were all that were left in
// the pin budget) - enough for green/red, but no yellow/overstay for Bay 2
// unless a pin is freed up later.
const uint8_t LED2_R = 17;
const uint8_t LED2_G = 23;

const uint8_t PLATE_SDA_PIN = 21;
const uint8_t PLATE_SCL_PIN = 22;
const uint8_t BMP180_ADDR   = 0x77;

// Exit plate - separate I2C bus (Wire1), since BMP180's address is fixed
// and two can't share one bus.
const uint8_t PLATE2_SDA_PIN = 26;
const uint8_t PLATE2_SCL_PIN = 32;
TwoWire ExitWire = TwoWire(1);

const uint8_t GATE_SERVO_PIN = 18;
const uint8_t GATE_LED_PIN   = 16; // RX2 - flashes while either plate is pressed
const uint32_t GATE_LED_FLASH_MS = 200;
bool gateLedState = false;
uint32_t gateLedLastToggle = 0;

const uint8_t  BUZZER_PIN      = 19;
const uint8_t  BUZZER2_PIN     = 5;
const uint16_t BUZZER_FREQ_HZ  = 2000; // audible alarm pitch
bool buzzerOn  = false; // tracks current state so we only log on a change, not every cycle
bool buzzer2On = false;

const float NEAR_CM = 40.0; // at/below this distance a sensor counts as "blocked"

const uint32_t READ_INTERVAL_MS = 150;
uint32_t lastReadAt = 0;

// --- overstay tuning (each bay tracks its own independently) ---
const uint32_t OVERSTAY_MS = 10000; // how long all-blocked must hold before it's an "overstay"
uint32_t blockedSince = 0;          // Bay 1: millis() when all-blocked started; 0 = not currently blocked
uint32_t bay2BlockedSince = 0;      // Bay 2: same, independent timer

// --- pressure plate tuning (entry) ---
const int16_t  PRESSURE_DELTA_THRESHOLD = 120; // raw ADC counts above baseline = "pressed"
const uint32_t PLATE_DEBOUNCE_MS        = 300; // must be stable this long before it counts
int16_t plateBaseline = 0;
bool plateCandidateActive = false;
uint32_t plateCandidateSince = 0;
bool plateActive = false;

// --- pressure plate tuning (exit) - same thresholds, independent state ---
int16_t plateExitBaseline = 0;
bool plateExitCandidateActive = false;
uint32_t plateExitCandidateSince = 0;
bool plateExitActive = false;

// --- gate FSM ---
const int      GATE_CLOSED_ANGLE = 0;
const int      GATE_OPEN_ANGLE   = 90;
const uint32_t GATE_HOLD_MS      = 6000; // how long the gate stays open once the plate clears
Servo gateServo;
bool gateOpen = false;
bool gateForExit = false;  // which plate opened it, so we know which one to wait on
uint32_t gateHoldUntil = 0; // 0 = not yet counting down

// Fires one HC-SR04 (given its own TRIG pin) and returns distance in cm, or
// -1 if nothing echoed back within range. Always reads on ECHO_SHARED_PIN -
// safe because callers only ever trigger one sensor at a time. Same math as
// before: sound takes ~58us per cm of round trip.
float readDistanceCm(uint8_t trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t durationUs = pulseIn(ECHO_SHARED_PIN, HIGH, 25000UL); // ~430cm max range
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

// Bay 2's LED - only Red/Green available (see LED2_R/LED2_G above).
void setColor2(bool r, bool g) {
  digitalWrite(LED2_R, r);
  digitalWrite(LED2_G, g);
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
    Serial.println("[BUZZER1] ON");
  } else {
    noTone(BUZZER_PIN);
    Serial.println("[BUZZER1] OFF");
  }
}

// Bay 2's buzzer - same tone()/noTone() approach as Bay 1's, own pin.
void setBuzzer2(bool on) {
  if (on == buzzer2On) return;
  buzzer2On = on;
  if (on) {
    tone(BUZZER2_PIN, BUZZER_FREQ_HZ);
    Serial.println("[BUZZER2] ON");
  } else {
    noTone(BUZZER2_PIN);
    Serial.println("[BUZZER2] OFF");
  }
}

// --- BMP180 minimal raw-pressure driver ---
// We only need a relative change to detect "something is pressing the
// plate", not a calibrated absolute pressure, so this skips the full
// temperature-compensation sequence and just reads the raw pressure
// register directly.
int16_t readRawPressure(TwoWire &bus) {
  bus.beginTransmission(BMP180_ADDR);
  bus.write(0xF4);
  bus.write(0x34); // start pressure measurement, OSS = 0
  bus.endTransmission();
  delay(5);

  bus.beginTransmission(BMP180_ADDR);
  bus.write(0xF6);
  bus.endTransmission(false);
  bus.requestFrom(BMP180_ADDR, (uint8_t)2);
  if (bus.available() < 2) return 0;
  uint8_t msb = bus.read();
  uint8_t lsb = bus.read();
  return (int16_t)((msb << 8) | lsb);
}

// Updates plateActive with the same debounce pattern used elsewhere: a
// candidate state must hold steady for PLATE_DEBOUNCE_MS before it commits.
void servicePlate() {
  int16_t raw = readRawPressure(Wire);
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

// Exit plate - identical logic to servicePlate(), own bus/state.
void servicePlateExit() {
  int16_t raw = readRawPressure(ExitWire);
  int16_t delta = raw - plateExitBaseline;
  bool rawActive = delta > PRESSURE_DELTA_THRESHOLD;

  if (rawActive != plateExitCandidateActive) {
    plateExitCandidateActive = rawActive;
    plateExitCandidateSince = millis();
  }
  if ((millis() - plateExitCandidateSince) >= PLATE_DEBOUNCE_MS) {
    plateExitActive = plateExitCandidateActive;
  }
}

// Opens the gate when EITHER plate is pressed; once that same plate clears,
// holds open for GATE_HOLD_MS before closing again. If both were pressed at
// once, entry takes priority (an edge case, not something either plate's
// normal use should trigger).
void serviceGate() {
  uint32_t now = millis();

  if (!gateOpen) {
    if (plateActive) {
      gateServo.write(GATE_OPEN_ANGLE);
      gateOpen = true;
      gateForExit = false;
      gateHoldUntil = 0;
      Serial.println("[GATE] Opening (entry)");
    } else if (plateExitActive) {
      gateServo.write(GATE_OPEN_ANGLE);
      gateOpen = true;
      gateForExit = true;
      gateHoldUntil = 0;
      Serial.println("[GATE] Opening (exit)");
    }
    return;
  }

  // Gate is open: start the hold timer once the relevant plate is released,
  // then close once that hold time has passed.
  bool relevantActive = gateForExit ? plateExitActive : plateActive;
  if (!relevantActive && gateHoldUntil == 0) {
    gateHoldUntil = now + GATE_HOLD_MS;
  }
  if (gateHoldUntil != 0 && now >= gateHoldUntil) {
    gateServo.write(GATE_CLOSED_ANGLE);
    gateOpen = false;
    gateHoldUntil = 0;
    Serial.println("[GATE] Closing");
  }
}

// Flashes gateLed (toggles every GATE_LED_FLASH_MS) for the entire time the
// gate is open - triggered by a plate press, continues through the post-
// clear hold, stops the moment the gate actually closes.
void serviceGateLed() {
  if (!gateOpen) {
    if (gateLedState) {
      gateLedState = false;
      digitalWrite(GATE_LED_PIN, LOW);
    }
    return;
  }

  uint32_t now = millis();
  if (now - gateLedLastToggle >= GATE_LED_FLASH_MS) {
    gateLedLastToggle = now;
    gateLedState = !gateLedState;
    digitalWrite(GATE_LED_PIN, gateLedState);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ECHO_SHARED_PIN, INPUT);

  pinMode(TOP_TRIG_PIN, OUTPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);

  pinMode(BAY2_TOP_TRIG_PIN, OUTPUT);
  pinMode(BAY2_LEFT_TRIG_PIN, OUTPUT);
  pinMode(BAY2_RIGHT_TRIG_PIN, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED2_R, OUTPUT);
  pinMode(LED2_G, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUZZER2_PIN, OUTPUT);
  pinMode(GATE_LED_PIN, OUTPUT);

  Wire.begin(PLATE_SDA_PIN, PLATE_SCL_PIN);
  plateBaseline = readRawPressure(Wire); // ambient reading, no vehicle on the plate yet

  ExitWire.begin(PLATE2_SDA_PIN, PLATE2_SCL_PIN);
  plateExitBaseline = readRawPressure(ExitWire);

  gateServo.setPeriodHertz(50);
  gateServo.attach(GATE_SERVO_PIN, 500, 2500);
  gateServo.write(GATE_CLOSED_ANGLE);
}

void loop() {
  uint32_t now = millis();
  if (now - lastReadAt < READ_INTERVAL_MS) return;
  lastReadAt = now;

  servicePlate();
  servicePlateExit();
  serviceGate();
  serviceGateLed();

  float topDistance   = readDistanceCm(TOP_TRIG_PIN);
  float leftDistance  = readDistanceCm(LEFT_TRIG_PIN);
  float rightDistance = readDistanceCm(RIGHT_TRIG_PIN);

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

  Serial.printf("Bay1 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED %s\n",
                topDistance,   topBlocked   ? "blocked" : "clear",
                leftDistance,  leftBlocked  ? "blocked" : "clear",
                rightDistance, rightBlocked ? "blocked" : "clear",
                ledLabel);

  // --- Bay 2: same rule as Bay 1 (including overstay -> yellow), on its own
  // LED, its own timer, and its own buzzer. ---
  float bay2TopDistance   = readDistanceCm(BAY2_TOP_TRIG_PIN);
  float bay2LeftDistance  = readDistanceCm(BAY2_LEFT_TRIG_PIN);
  float bay2RightDistance = readDistanceCm(BAY2_RIGHT_TRIG_PIN);

  bool bay2TopBlocked   = isBlocked(bay2TopDistance);
  bool bay2LeftBlocked  = isBlocked(bay2LeftDistance);
  bool bay2RightBlocked = isBlocked(bay2RightDistance);
  bool bay2AllBlocked   = bay2TopBlocked && bay2LeftBlocked && bay2RightBlocked;

  const char *led2Label;
  if (bay2AllBlocked) {
    if (bay2BlockedSince == 0) bay2BlockedSince = now;

    bool bay2Overstay = (now - bay2BlockedSince) >= OVERSTAY_MS;
    if (bay2Overstay) {
      setColor2(true, true);   // yellow
      setBuzzer2(true);
      led2Label = "YELLOW (overstay)";
    } else {
      setColor2(true, false);  // red
      setBuzzer2(false);
      led2Label = "RED";
    }
  } else {
    bay2BlockedSince = 0;
    setColor2(false, true);    // green
    setBuzzer2(false);
    led2Label = "GREEN";
  }

  Serial.printf("Bay2 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED2 %s\n",
                bay2TopDistance,   bay2TopBlocked   ? "blocked" : "clear",
                bay2LeftDistance,  bay2LeftBlocked  ? "blocked" : "clear",
                bay2RightDistance, bay2RightBlocked ? "blocked" : "clear",
                led2Label);
}
