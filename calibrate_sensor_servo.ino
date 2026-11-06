/*
  calibrate_sensor_servo.ino - find the servo angle at which the HC-SR04 looks STRAIGHT AHEAD

  PIN MAP (same as every sketch): HC-SR04 TRIG=5 ECHO=35 (through the 1k/2k divider), sensor servo=14

  HOW TO USE
    1. Put the robot facing a FLAT wall (or a big book on its edge) about 30 cm away.
       The wall must be square to the robot: measure from the left wheel and from the right wheel
       to the wall with a ruler. The two distances must be equal.
    2. Make sure the sensor horn is mounted roughly straight ahead (servo at 90 degrees, see test_servos 'c').
    3. Upload, open Serial Monitor at 115200 and send  g.
    4. The servo scans from 60 to 120 degrees and prints the distance at each angle.
       The distance is SMALLEST when the sensor faces the wall squarely, so the centre of the
       smallest readings is the angle where the sensor really looks straight ahead.
    5. Result between 86 and 94: leave everything as it is.
       Otherwise either move the horn one tooth on the spline and repeat, or keep the printed
       SENSOR_CENTER_DEG and tell the movement code to use it as "straight ahead".
*/

#include <Arduino.h>

const int PIN_TRIG = 5;
const int PIN_ECHO = 35;
const int PIN_SERVO_SENSOR = 14;

const int SENSOR_MIN = 10, SENSOR_MAX = 170;
const int US_MIN = 500, US_MAX = 2400;
const int SERVO_FREQ = 50, SERVO_BITS = 14;
const int CH_SENSOR = 2;

const int SCAN_FROM = 60, SCAN_TO = 120, SCAN_STEP = 3;
const int READS_PER_ANGLE = 3;
const int SETTLE_MS = 350;                  // wait for the servo to stop before reading
const float PLATEAU_CM = 1.0f;              // readings within 1 cm of the minimum count as "facing the wall"

int currentDeg = 90;

void servoInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO_SENSOR, SERVO_FREQ, SERVO_BITS);
#else
  ledcSetup(CH_SENSOR, SERVO_FREQ, SERVO_BITS);
  ledcAttachPin(PIN_SERVO_SENSOR, CH_SENSOR);
#endif
}

void servoWrite(int deg) {
  currentDeg = constrain(deg, SENSOR_MIN, SENSOR_MAX);
  int us = map(currentDeg, 0, 180, US_MIN, US_MAX);
  uint32_t duty = (uint32_t)us * (1UL << SERVO_BITS) / 20000UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO_SENSOR, duty);
#else
  ledcWrite(CH_SENSOR, duty);
#endif
}

void moveSlowlyTo(int target) {
  while (currentDeg != target) {
    int d = target - currentDeg;
    servoWrite(currentDeg + ((abs(d) < 2) ? d : (d > 0 ? 2 : -2)));
    delay(20);
  }
}

float readOnceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) return -1.0f;
  return us * 0.0343f / 2.0f;
}

float readMedianCm() {
  float v[READS_PER_ANGLE];
  int m = 0;
  for (int i = 0; i < READS_PER_ANGLE; i++) {
    float r = readOnceCm();
    if (r > 0) v[m++] = r;
    delay(30);
  }
  if (m == 0) return -1.0f;
  for (int i = 1; i < m; i++) {
    float key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  return v[m / 2];
}

void runScan() {
  const int N = (SCAN_TO - SCAN_FROM) / SCAN_STEP + 1;
  float dist[N];
  Serial.println("Scanning... keep the robot still.");
  moveSlowlyTo(SCAN_FROM);
  delay(600);

  for (int i = 0; i < N; i++) {
    int ang = SCAN_FROM + i * SCAN_STEP;
    servoWrite(ang);
    delay(SETTLE_MS);
    dist[i] = readMedianCm();
    if (dist[i] < 0) Serial.printf("%3d deg   no echo\n", ang);
    else             Serial.printf("%3d deg   %5.1f cm\n", ang, dist[i]);
  }

  float minD = 1e9f;
  int valid = 0;
  for (int i = 0; i < N; i++) {
    if (dist[i] > 0) {
      valid++;
      if (dist[i] < minD) minD = dist[i];
    }
  }
  if (valid < 5) {
    Serial.println("Not enough valid readings. Put a flat wall 20-40 cm in front of the sensor and try again.");
    moveSlowlyTo(90);
    return;
  }

  float sum = 0;
  int cnt = 0;
  for (int i = 0; i < N; i++) {
    if (dist[i] > 0 && dist[i] <= minD + PLATEAU_CM) {
      sum += SCAN_FROM + i * SCAN_STEP;
      cnt++;
    }
  }
  int center = (int)lroundf(sum / cnt);
  Serial.printf("\nSmallest distance: %.1f cm.\n", minD);
  Serial.printf("RESULT: SENSOR_CENTER_DEG = %d   (offset from 90 = %d degrees)\n", center, center - 90);
  if (abs(center - 90) <= 4) Serial.println("Good: the sensor looks straight ahead at 90 degrees. Change nothing.");
  else Serial.println("The sensor is not straight at 90. Move the horn one tooth, or use this number as 'straight ahead'.");
  Serial.println("If the robot was not square to the wall, this result is wrong. Check the two wheel distances.");
  moveSlowlyTo(90);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  servoInit();
  servoWrite(90);
  Serial.println("Sensor-servo calibration. Face a flat wall at 30 cm, square to the robot, then send  g");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  delay(20);
  while (Serial.available()) Serial.read();
  if (c == 'g') runScan();
}
