#include <Servo.h>

Servo ESC;

// ---- Pins ----
const int ESC_PIN  = 8;
const int TRIG_PIN = 22;
const int ECHO_PIN = 23;

const int HX_DATA = 24;
const int HX_CLK  = 25;

// ---- Ultrasonic settings ----
const unsigned long TIMEOUT_US = 25000;  // ~4.3 m max range

// ---- Distance thresholds (cm) ----
const float STOP_DIST_CM = 40.0;
const float SLOW_DIST_CM = 90.0;

// ---- ESC pulse widths (microseconds) -- TUNE THESE ----
const int STOP_US = 1000;  // ESC minimum = motor off
const int SLOW_US = 1150;  // placeholder for ~1 mph // 1125 is cutoff from off to spinning
const int FAST_US = 1300;  // placeholder for ~2 mph

const int RAMP_STEP_US = 5;  // max increase per loop (smooth acceleration)
int currentUs = STOP_US;     // pulse width being sent to the ESC right now

// Load Cell variables
long offset = 0;              // raw reading with the empty wood sheet on (set by tare)
float COUNTS_PER_LB = 33.1;    // calibration factor -- set by typing 'c'
const float KNOWN_WEIGHT_LB = 1.36;  // weight of your test object -- change this

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW); // LOW tells us no data read

  pinMode(HX_CLK, OUTPUT);
  pinMode(HX_DATA, INPUT);
  digitalWrite(HX_CLK, LOW);   // LOW keeps the HX711 awake
  Serial.println("Taring... keep the scale empty");
  offset = hxAverage(10);
  Serial.print("Offset: ");
  Serial.println(offset);
  Serial.println("Commands: t = re-tare (empty), c = calibrate (known weight on)");

  ESC.attach(ESC_PIN, 1000, 2000);
  ESC.writeMicroseconds(STOP_US);  // arm the ESC at zero throttle
  delay(3000);                     // give the ESC time to arm
}

// Fires one ping and returns the distance in cm, or -1 if no echo
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  if (duration == 0) {
    return -1.0;  // -1 means "no echo"
  }
  return duration / 58.0;
}

// True when the HX711 has a new reading waiting
bool hxReady() {
  return digitalRead(HX_DATA) == LOW;
}

// Clock out one 24-bit reading
long hxReadRaw() {
  long value = 0;

  for (int i = 0; i < 24; i++) {
    digitalWrite(HX_CLK, HIGH);             // pulse: next bit appears on DATA
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX_DATA);  // add that bit to the end
    digitalWrite(HX_CLK, LOW);
    delayMicroseconds(1);
  }

  // 25th pulse: channel A, gain 128 for the next reading
  digitalWrite(HX_CLK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX_CLK, LOW);

  // The HX711 sends a signed 24-bit number; convert it to a signed 32-bit long
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return value;
}

// Average several readings (used for taring)
long hxAverage(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    while (!hxReady()) {}   // wait for the next reading (~100 ms each)
    sum += hxReadRaw();
  }
  return sum / n;
}

void loop() {
  // float distanceCm = readDistanceCm();

  // // 1. Pick the target speed level
  // int targetUs;
  // if (distanceCm < 0) {
  //   targetUs = FAST_US;  // no echo: nothing within ~4.3 m
  // } else if (distanceCm < STOP_DIST_CM) {
  //   targetUs = STOP_US;
  // } else if (distanceCm < SLOW_DIST_CM) {
  //   targetUs = SLOW_US;
  // } else {
  //   targetUs = FAST_US;
  // }

  // // 2. Slow down or stop immediately; speed up gradually
  // if (targetUs < currentUs) {
  //   currentUs = targetUs;
  // } else {
  //   currentUs = min(currentUs + RAMP_STEP_US, targetUs);
  // }
  // ESC.writeMicroseconds(currentUs);

  // 5. Calibration commands from the Serial Monitor
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 't' || cmd == 'c') {
      ESC.writeMicroseconds(STOP_US);  // stop the motor while averaging
      currentUs = STOP_US;             // so it ramps back up gently afterward
    }

    if (cmd == 't') {
      Serial.println("Taring... keep the scale empty");
      offset = hxAverage(20);
      Serial.print("New offset: ");
      Serial.println(offset);
    }
    else if (cmd == 'c') {
      Serial.println("Calibrating... keep the known weight still");
      long net = hxAverage(20) - offset;
      if (net == 0) {
        Serial.println("No change detected -- is the weight on the scale?");
      } else {
        COUNTS_PER_LB = net / KNOWN_WEIGHT_LB;
        Serial.print("COUNTS_PER_LB = ");
        Serial.println(COUNTS_PER_LB, 1);
        Serial.println("Copy this number into the code to keep it.");
      }
    }
  }

  // 3. HX711 Load Cell
  if (hxReady()) {             // only read when a new value is waiting
    long raw = hxReadRaw();
    long net = raw - offset;
    float lb = net / COUNTS_PER_LB;

    Serial.print("Raw: ");
    Serial.println(raw);
    Serial.print("Net counts: ");
    Serial.print(net);
    Serial.print("   Weight: ");
    Serial.print(lb, 2);
    Serial.println(" lb");
  }    
  
  // 4. Report

  // if (distanceCm < 0) {
  //   Serial.print("Distance: no echo");
  // } else {
  //   Serial.print("Distance: ");
  //   Serial.print(distanceCm, 1);
  //   Serial.print(" cm");
  // }
  // Serial.print("   Target: ");
  // Serial.print(targetUs);
  // Serial.print(" us   ESC: ");
  // Serial.print(currentUs);
  // Serial.println(" us");

  delay(50);
}

