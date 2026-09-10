#include <Arduino.h>

#define TRIG_PIN 5
#define ECHO_PIN 18

// put function declarations here:
int myFunction(int, int);

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

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  enum ParkingState {
  VACANT,
  IN_PROCESS,
  OCCUPIED
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



delay(1000);

}

