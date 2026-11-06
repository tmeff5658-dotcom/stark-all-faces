/*
  test_servos.ino - both SG90 servos (sensor servo and screen servo)

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire each servo: brown/black -> GND, red -> 5V from the Buck (NOT from the ESP32 3.3V pin),
       orange/yellow (signal) -> GPIO14 (sensor servo) or GPIO13 (screen servo). GND must be common.
       Put a 470-1000 uF capacitor across the 5V line near the servos.
    2. Upload, open Serial Monitor at 115200, and send one letter:
         c = centre both servos at 90 degrees
         s = sweep the SENSOR servo slowly between its limits
         d = sweep the SCREEN servo slowly between its limits
         3 = the obstacle scan: sensor servo to 30, 90, 150 degrees, 1 s each
         x = (or any key during a move) stop where it is
    3. Expected: smooth movement, no buzzing at the ends, no ESP32 reset.
       Buzzing at an end = the horn hits something: reduce that servo's limits below.
    4. Mount the horns so that 90 degrees = straight ahead (sensor) and the screen faces straight back.
*/

#include <Arduino.h>

const int PIN_SERVO_SENSOR = 14;
const int PIN_SERVO_SCREEN = 13;

// Safe limits (never go to 0 or 180: the servo pushes against its stop and draws a lot of current)
const int SENSOR_MIN = 10, SENSOR_MAX = 170;
const int SCREEN_MIN = 20, SCREEN_MAX = 160;

const int US_MIN = 500, US_MAX = 2400;         // SG90 pulse range
const int SERVO_FREQ = 50, SERVO_BITS = 14;
const int CH_SENSOR = 2, CH_SCREEN = 3;        // LEDC channels (core 2.x); motors use 0 and 1
const int STEP_DEG = 2, STEP_MS = 25;

int sensorDeg = 90;
int screenDeg = 90;

void servoInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO_SENSOR, SERVO_FREQ, SERVO_BITS);
  ledcAttach(PIN_SERVO_SCREEN, SERVO_FREQ, SERVO_BITS);
#else
  ledcSetup(CH_SENSOR, SERVO_FREQ, SERVO_BITS);
  ledcAttachPin(PIN_SERVO_SENSOR, CH_SENSOR);
  ledcSetup(CH_SCREEN, SERVO_FREQ, SERVO_BITS);
  ledcAttachPin(PIN_SERVO_SCREEN, CH_SCREEN);
#endif
}

void servoWrite(int pin, int channel, int deg, int lo, int hi) {
  deg = constrain(deg, lo, hi);
  int us = map(deg, 0, 180, US_MIN, US_MAX);
  uint32_t duty = (uint32_t)us * (1UL << SERVO_BITS) / 20000UL;   // 20 ms period
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void writeSensor(int deg) { sensorDeg = constrain(deg, SENSOR_MIN, SENSOR_MAX); servoWrite(PIN_SERVO_SENSOR, CH_SENSOR, sensorDeg, SENSOR_MIN, SENSOR_MAX); }
void writeScreen(int deg) { screenDeg = constrain(deg, SCREEN_MIN, SCREEN_MAX); servoWrite(PIN_SERVO_SCREEN, CH_SCREEN, screenDeg, SCREEN_MIN, SCREEN_MAX); }

// Any key pressed = stop.
bool abortRequested() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    Serial.println("STOPPED");
    return true;
  }
  return false;
}

bool waitMs(int ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) {
    if (abortRequested()) return true;
    delay(5);
  }
  return false;
}

// Moves one servo slowly to target. Returns false if aborted.
bool moveTo(bool sensor, int target) {
  int lo = sensor ? SENSOR_MIN : SCREEN_MIN;
  int hi = sensor ? SENSOR_MAX : SCREEN_MAX;
  target = constrain(target, lo, hi);
  int &cur = sensor ? sensorDeg : screenDeg;
  while (cur != target) {
    int d = target - cur;
    cur += (abs(d) < STEP_DEG) ? d : (d > 0 ? STEP_DEG : -STEP_DEG);
    if (sensor) writeSensor(cur); else writeScreen(cur);
    if (waitMs(STEP_MS)) return false;
  }
  return true;
}

void sweep(bool sensor) {
  int lo = sensor ? SENSOR_MIN : SCREEN_MIN;
  int hi = sensor ? SENSOR_MAX : SCREEN_MAX;
  Serial.printf("Sweeping the %s servo between %d and %d degrees...\n", sensor ? "SENSOR" : "SCREEN", lo, hi);
  if (!moveTo(sensor, lo)) return;
  if (!moveTo(sensor, hi)) return;
  if (!moveTo(sensor, lo)) return;
  moveTo(sensor, 90);
  Serial.println("Done.");
}

void obstacleScan() {
  const int angles[3] = { 30, 90, 150 };
  Serial.println("Obstacle scan: sensor servo to 30, 90, 150 degrees, 1 s each.");
  for (int i = 0; i < 3; i++) {
    if (!moveTo(true, angles[i])) return;
    Serial.printf("at %d degrees\n", angles[i]);
    if (waitMs(1000)) return;
  }
  moveTo(true, 90);
  Serial.println("Done.");
}

void setup() {
  Serial.begin(115200);
  servoInit();
  writeSensor(90);
  writeScreen(90);
  Serial.println("Servo test. Commands: c = centre, s = sweep sensor, d = sweep screen, 3 = obstacle scan, x = stop");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  delay(20);
  while (Serial.available()) Serial.read();

  switch (c) {
    case 'c': Serial.println("Centre"); moveTo(true, 90); moveTo(false, 90); break;
    case 's': sweep(true); break;
    case 'd': sweep(false); break;
    case '3': obstacleScan(); break;
    default: break;
  }
}
