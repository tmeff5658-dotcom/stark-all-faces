/*
  motor_test_v2.ino - motor test + calibration log (ESP32 + L298N)

  Board : ESP32 Dev Module (ESP32-WROOM-32, 38 pins)
  Core  : works with arduino-esp32 2.x and 3.x

  Convention
    RIGHT / LEFT = as seen from BEHIND the robot, looking in the direction of motion.
    FORWARD      = toward the caster wheel and the HC-SR04.
    Right motor  = L298N channel A (IN1, IN2, ENA)
    Left  motor  = L298N channel B (IN3, IN4, ENB)

  HOW TO USE
    1. Edit ONLY section 4 (CALIBRATION) between runs, then upload.
    2. Open Serial Monitor at 115200 baud.
    3. Send a command letter, watch the robot, then write the result in the
       log table at the bottom of this file (or in the Doc).
    4. Change ONE number per run.

  COMMANDS
    r = ramp test, right wheel (find MIN_PWM_RIGHT)
    l = ramp test, left wheel  (find MIN_PWM_LEFT)
    b = straight run, both wheels, RUN_MS   (robot on the floor, clear space ahead)
    p = spin in place to the right, RUN_MS  (robot on the floor, 30 cm free all around)
    q = spin in place to the left,  RUN_MS
    s = stop      ? = print current settings
    Any key pressed DURING a test aborts it and stops the motors.

  BEFORE POWERING THE MOTORS
    - ENA and ENB jumpers on the L298N board removed
    - 10k pull-down resistor from ENA and ENB to GND
    - common GND between battery, L298N and ESP32
    - for r / l tests the robot may be lifted; for b / p / q it must be on the floor
*/

#include <Arduino.h>

// ===================== 1) PINS (edit only if the wiring changes) =====================
const int R_IN1 = 25;
const int R_IN2 = 26;
const int R_EN  = 27;   // set to 2 if the ENA wire was NOT moved
const int L_IN1 = 32;
const int L_IN2 = 33;
const int L_EN  = 18;   // set to 15 if the ENB wire was NOT moved

// ===================== 2) DIRECTION FIX =====================
// Change to true ONLY if that wheel turns backward when the code says "forward".
// Do NOT also swap the motor wires, or the two fixes cancel each other.
const bool INVERT_RIGHT = false;
const bool INVERT_LEFT  = false;

// ===================== 3) HARD LIMITS (do not raise: motors are rated 3.7 V) =====================
const int PWM_FREQ = 5000;   // Hz
const int PWM_BITS = 8;      // duty range 0..255
const int PWM_MAX  = 153;    // 60 %

// ===================== 4) CALIBRATION (edit between runs, then record) =====================
// MIN_PWM_* : lowest duty at which that wheel moves the robot smoothly ON THE FLOOR.
//             The values below are PLACEHOLDERS until you measure them with r / l.
const int   MIN_PWM_RIGHT = 90;
const int   MIN_PWM_LEFT  = 90;

// BASE_PWM  : test speed for b / p / q (must be <= PWM_MAX).
const int   BASE_PWM      = 110;

// TRIM_*    : multiplier on each wheel's duty. Keep between 0.80 and 1.00.
//             Robot drifts RIGHT  -> lower TRIM_LEFT  (e.g. 0.97)
//             Robot drifts LEFT   -> lower TRIM_RIGHT (e.g. 0.97)
const float TRIM_RIGHT    = 1.00f;
const float TRIM_LEFT     = 1.00f;

// RUN_MS    : duration of b / p / q.
const int   RUN_MS        = 2000;

// Optional kick-start: a short burst at KICK_PWM to get the wheels moving from rest.
const int   KICK_PWM      = 150;
const int   KICK_MS       = 0;      // 0 = disabled

// ===================== 5) RAMP TEST (r / l) =====================
const int RAMP_START   = 60;
const int RAMP_STEP    = 10;
const int RAMP_STEP_MS = 600;

// ===================== internals =====================
const int R_CH = 0;   // LEDC channels, used by core 2.x only
const int L_CH = 1;
int runNo = 0;

// ---------------- PWM wrappers (core 2.x / 3.x) ----------------
void pwmInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(R_EN, PWM_FREQ, PWM_BITS);
  ledcAttach(L_EN, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(R_CH, PWM_FREQ, PWM_BITS);
  ledcAttachPin(R_EN, R_CH);
  ledcSetup(L_CH, PWM_FREQ, PWM_BITS);
  ledcAttachPin(L_EN, L_CH);
#endif
}

void pwmWrite(int pin, int channel, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// ---------------- Motor control ----------------
// speed: -PWM_MAX .. +PWM_MAX  (positive = forward, 0 = stop)
void setMotor(int in1, int in2, int enPin, int enCh, bool invert, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);

  if (speed == 0) {
    pwmWrite(enPin, enCh, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  bool forward = (speed > 0);
  if (invert) forward = !forward;

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  pwmWrite(enPin, enCh, abs(speed));
}

void setRight(int s) { setMotor(R_IN1, R_IN2, R_EN, R_CH, INVERT_RIGHT, s); }
void setLeft(int s)  { setMotor(L_IN1, L_IN2, L_EN, L_CH, INVERT_LEFT,  s); }
void stopAll()       { setRight(0); setLeft(0); }

int sgn(int v) { return (v > 0) - (v < 0); }

// ---------------- Test helpers ----------------
// Returns true if a key was pressed (and stops the motors).
bool abortRequested() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    stopAll();
    Serial.println("ABORTED (no log line printed)");
    return true;
  }
  return false;
}

// Waits ms milliseconds; returns true if aborted.
bool waitMs(int ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) {
    if (abortRequested()) return true;
    delay(10);
  }
  return false;
}

// Applies trim to a base duty and clamps to 0..PWM_MAX.
int scaledDuty(int base, float trim) {
  int d = (int)lroundf(base * trim);
  return constrain(d, 0, PWM_MAX);
}

// Runs both wheels for ms. Signed duties (+ = forward).
// Returns true if finished, false if aborted.
bool runBoth(int dutyR, int dutyL, int ms) {
  if (KICK_MS > 0) {
    setRight(sgn(dutyR) * KICK_PWM);
    setLeft(sgn(dutyL) * KICK_PWM);
    if (waitMs(KICK_MS)) return false;
  }
  setRight(dutyR);
  setLeft(dutyL);
  if (waitMs(ms)) return false;
  stopAll();
  return true;
}

void warnIfBelowMin(int dutyR, int dutyL) {
  if (dutyR < MIN_PWM_RIGHT) {
    Serial.printf("WARNING: dutyR=%d is below MIN_PWM_RIGHT=%d, the right wheel may not move.\n", dutyR, MIN_PWM_RIGHT);
  }
  if (dutyL < MIN_PWM_LEFT) {
    Serial.printf("WARNING: dutyL=%d is below MIN_PWM_LEFT=%d, the left wheel may not move.\n", dutyL, MIN_PWM_LEFT);
  }
}

// One CSV line per finished run: copy it as is into your spreadsheet.
void printLogLine(const char* test, int dutyR, int dutyL, int ms) {
  runNo++;
  Serial.printf("CSV,%d,%s,%d,%d,%d,%.2f,%.2f,%d,%d,%d\n",
                runNo, test, MIN_PWM_RIGHT, MIN_PWM_LEFT, BASE_PWM,
                TRIM_RIGHT, TRIM_LEFT, dutyR, dutyL, ms);
  Serial.println("NOW RECORD BY HAND: battery voltage, distance (cm), drift side (L/R/none), notes.");
}

void printConfig() {
  Serial.println("\n--- CURRENT SETTINGS ---");
  Serial.printf("PWM_MAX=%d  MIN_PWM_RIGHT=%d  MIN_PWM_LEFT=%d  BASE_PWM=%d\n",
                PWM_MAX, MIN_PWM_RIGHT, MIN_PWM_LEFT, BASE_PWM);
  Serial.printf("TRIM_RIGHT=%.2f  TRIM_LEFT=%.2f  RUN_MS=%d  KICK_PWM=%d  KICK_MS=%d\n",
                TRIM_RIGHT, TRIM_LEFT, RUN_MS, KICK_PWM, KICK_MS);
  Serial.printf("INVERT_RIGHT=%d  INVERT_LEFT=%d\n", (int)INVERT_RIGHT, (int)INVERT_LEFT);
  Serial.println("CSV columns: CSV,run,test,minR,minL,base,trimR,trimL,dutyR,dutyL,ms");
  if (BASE_PWM > PWM_MAX) {
    Serial.println("WARNING: BASE_PWM is above PWM_MAX, it will be clamped.");
  }
}

// ---------------- Tests ----------------
void rampTest(bool right) {
  const char* name = right ? "RIGHT" : "LEFT";
  Serial.printf("\n== %s wheel: forward ramp ==\n", name);
  Serial.println("Note the duty where the wheel STARTS to turn (on the floor: where the robot starts to move).");

  int d = RAMP_START;
  while (true) {
    int duty = min(d, PWM_MAX);
    if (right) setRight(duty); else setLeft(duty);
    Serial.printf("duty = %d  (%d %%)\n", duty, duty * 100 / 255);
    if (waitMs(RAMP_STEP_MS)) return;
    if (duty == PWM_MAX) break;
    d += RAMP_STEP;
  }

  stopAll();
  Serial.printf("Done. Put the value you noted into MIN_PWM_%s and upload again.\n", name);
}

void straightTest() {
  int dr = scaledDuty(BASE_PWM, TRIM_RIGHT);
  int dl = scaledDuty(BASE_PWM, TRIM_LEFT);
  Serial.printf("\n== STRAIGHT: dutyR=%d dutyL=%d for %d ms (on the floor, clear space ahead) ==\n", dr, dl, RUN_MS);
  warnIfBelowMin(dr, dl);
  if (runBoth(dr, dl, RUN_MS)) printLogLine("straight", dr, dl, RUN_MS);
}

void spinTest(bool toRight) {
  int dr = scaledDuty(BASE_PWM, TRIM_RIGHT);
  int dl = scaledDuty(BASE_PWM, TRIM_LEFT);
  Serial.printf("\n== SPIN %s: dutyR=%d dutyL=%d for %d ms (30 cm free all around) ==\n",
                toRight ? "RIGHT" : "LEFT", dr, dl, RUN_MS);
  warnIfBelowMin(dr, dl);
  // Spin right: left wheel forward, right wheel backward. Spin left: the opposite.
  int r = toRight ? -dr : dr;
  int l = toRight ?  dl : -dl;
  if (runBoth(r, l, RUN_MS)) printLogLine(toRight ? "spin_right" : "spin_left", dr, dl, RUN_MS);
  Serial.println("For spin tests also record the angle turned (degrees).");
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);

  pwmInit();
  stopAll();

  Serial.println("\nMotor test v2 ready.");
  printConfig();
  Serial.println("Commands: r | l = ramp   b = straight   p | q = spin right | left   s = stop   ? = settings");
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  delay(20);                                  // let the rest of the line arrive
  while (Serial.available()) Serial.read();   // drain it, so a newline does not abort the test

  switch (c) {
    case 'r': rampTest(true);    break;
    case 'l': rampTest(false);   break;
    case 'b': straightTest();    break;
    case 'p': spinTest(true);    break;
    case 'q': spinTest(false);   break;
    case 's': stopAll(); Serial.println("STOP"); break;
    case '?': printConfig();     break;
    default: break;                            // ignore anything else
  }
}

/*
  ===================== TEST LOG (copy to your notebook / spreadsheet) =====================
  Run | Test     | minR | minL | base | trimR | trimL | ms   | Battery V | Distance cm | Drift L/R/none | Angle deg | Notes
  ----+----------+------+------+------+-------+-------+------+-----------+-------------+----------------+-----------+------
   1  |          |      |      |      |       |       |      |           |             |                |           |
   2  |          |      |      |      |       |       |      |           |             |                |           |
   3  |          |      |      |      |       |       |      |           |             |                |           |
   4  |          |      |      |      |       |       |      |           |             |                |           |
   5  |          |      |      |      |       |       |      |           |             |                |           |

  Tip: Run numbers restart at 1 after every reset or upload, so keep your own numbering in the table.
*/
