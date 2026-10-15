#include <Servo.h>
Servo ESC;
int speed;

const int TRIG_PIN = 22;
const int ECHO_PIN = 23;
const unsigned long TIMEOUT_US = 25000;  // 4.3m max range

void setup() {
  // put your setup code here, to run once:
  ESC.attach(8, 1000, 2000);
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void loop() {
  // motor controlling, take in A0 pin to determine ESC pinout
  // speed = analogRead(A0);
  // speed = map(analogRead(A0), 0, 1023, 1000, 2000);
  // ESC.writeMicroseconds(speed);
  // delay(20);

  // set trigger to high for 10 us
  digitalWrite(TRIG_PIN, LOW); // low voltage
  delayMicroseconds(2); // wait
  digitalWrite(TRIG_PIN, HIGH); // high voltage
  delayMicroseconds(10); // wait 10 us
  digitalWrite(TRIG_PIN, LOW); // back to low

  // Measure how long Echo stays HIGH (0 = no echo before timeout)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  if (duration == 0) {
    Serial.println("No echo (out of range)");
  } else {
    float distanceCm = duration / 58.0; // dist calc
    Serial.print("Echo: "); // serial monitor outputs
    Serial.print(duration);
    Serial.print(" us   Distance: ");
    Serial.print(distanceCm, 1);
    Serial.println(" cm");
  }

  delay(50);  // let leftover echoes die out before the next ping
}
