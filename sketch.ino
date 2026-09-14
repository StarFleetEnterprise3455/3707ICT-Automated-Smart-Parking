#include <Arduino.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define OVERSTAY_LIMIT 20000
#define MAX_INVALID_READINGS 5

// put function declarations here:
int myFunction(int, int);

unsigned long occupiedStartTime = 0;
bool isOccupied = false;
bool buzzerOn = false;
int invalidReadingsCount = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Hello Smart Parking!");
}

void loop() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 seconds timeout
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  enum ParkingState {
  VACANT,
  IN_PROCESS,
  OCCUPIED,
  UNCERTAIN,
  FAULT
};


ParkingState currentState = VACANT;

if (distance > 1000)
{
  currentState = VACANT;
}
else if (distance > 500)
{
  currentState = IN_PROCESS;
}
else
{
  currentState = OCCUPIED;
}

if (currentState == OCCUPIED){

  if (!isOccupied) {
    isOccupied = true;
    occupiedStartTime = millis();
  } else {
    unsigned long occupiedDuration = millis() - occupiedStartTime;
    if (occupiedDuration > OVERSTAY_LIMIT && !buzzerOn) {
      buzzerOn = true;
      Serial.println("Buzzer ON: Overstay detected!");
      // Add code to activate the buzzer here
    }
  }
} else {
  isOccupied = false;
  occupiedStartTime = 0;
  buzzerOn = false;
  Serial.println("Buzzer OFF: Parking spot is vacant.");
  // Add code to deactivate the buzzer here
}

if (duration == 0) {
  invalidReadingsCount++;
  if (invalidReadingsCount >= MAX_INVALID_READINGS) {
    currentState = FAULT;
    Serial.println("Fault: Sensor readings are invalid.");
    // Add code to handle the fault condition here
  }
  else {
    currentState = UNCERTAIN;
    Serial.println("Uncertain: Sensor readings are inconsistent.");
  }
} else {

  invalidReadingsCount = 0; // Reset the count on a valid reading

}

if (currentState == FAULT) {
  Serial.println("Fault: Please check the sensor.");
  // Add code to handle the fault condition here
} else if (currentState == UNCERTAIN) {
  Serial.println("Uncertain: Please check the sensor readings.");
  // Add code to handle the uncertain condition here
}

delay(1000);

};