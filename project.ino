// Distance traffic-light indicator - ESP32 + 3x HC-SR04 + 1x RGB LED
//
// Left/right rule : <= 40cm = "blocked", > 40cm = "clear".
// Top rule        : <= 80cm = "in range" (this covers the whole 0-80cm band,
//                    including under 40cm), > 80cm = "clear".
//
// Combined LED rule (checked in this priority order):
//   1. RED    - all three sensors blocked (<=40cm) at the same time.
//   2. YELLOW - top sensor is within 80cm. This still applies even when top
//               itself is under 40cm - it only escalates to red once left
//               AND right are also under 40cm.
//   3. GREEN  - anything else (the default/safe state).

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

const float NEAR_CM = 40.0; // at/below this distance a sensor counts as "blocked"
const float FAR_CM  = 80.0; // top sensor: above this it's fully "clear" again

const uint32_t READ_INTERVAL_MS = 150;
uint32_t lastReadAt = 0;

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

// True whenever a valid reading falls at/under FAR_CM - used for the top
// sensor's "yellow" range, which spans the whole 0-80cm band (not just 40-80).
bool isInRange(float distanceCm) {
  return distanceCm >= 0 && distanceCm <= FAR_CM;
}

// Drives the 3 LED legs directly - true = that color's GPIO pin goes HIGH.
void setColor(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
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
}

void loop() {
  uint32_t now = millis();
  if (now - lastReadAt < READ_INTERVAL_MS) return;
  lastReadAt = now;

  float topDistance   = readDistanceCm(TOP_TRIG_PIN, TOP_ECHO_PIN);
  float leftDistance  = readDistanceCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float rightDistance = readDistanceCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  bool topBlocked    = isBlocked(topDistance);   // top <= 40cm - only used for the red condition
  bool topInRange    = isInRange(topDistance);   // top <= 80cm - drives yellow
  bool leftBlocked   = isBlocked(leftDistance);
  bool rightBlocked  = isBlocked(rightDistance);

  bool allBlocked = topBlocked && leftBlocked && rightBlocked;

  const char *ledLabel;
  if (allBlocked) {
    setColor(true, false, false);  // red - only when left/right catch up to top being blocked too
    ledLabel = "RED";
  } else if (topInRange) {
    setColor(true, true, false);   // yellow - covers top's whole 0-80cm band
    ledLabel = "YELLOW";
  } else {
    setColor(false, true, false);  // green
    ledLabel = "GREEN";
  }

  Serial.printf("Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED %s\n",
                topDistance,   topInRange   ? (topBlocked ? "blocked" : "in-range") : "clear",
                leftDistance,  leftBlocked  ? "blocked" : "clear",
                rightDistance, rightBlocked ? "blocked" : "clear",
                ledLabel);
}
