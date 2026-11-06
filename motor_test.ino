/*
  motor_test.ino - test each motor ALONE (ESP32 + L298N)

  Board : ESP32 Dev Module (ESP32-WROOM-32, 38 pins)
  Core  : works with arduino-esp32 2.x and 3.x

  Convention
    RIGHT / LEFT = as seen from BEHIND the robot, looking in the direction of motion.
    FORWARD      = toward the caster wheel and the HC-SR04.
    Right motor  = L298N channel A (IN1, IN2, ENA)
    Left  motor  = L298N channel B (IN3, IN4, ENB)

  Serial Monitor: 115200 baud (any line-ending setting).
    r = ramp test, right motor      l = ramp test, left motor
    b = both motors forward, 2 s    s = stop
    Any key pressed DURING a test aborts it and stops the motors.

  Before powering the motors:
    - ENA and ENB jumpers on the L298N board removed
    - 10k pull-down resistor from ENA and ENB to GND
    - common GND between battery, L298N and ESP32
    - robot lifted so the wheels spin in the air (except the 'b' test)
*/

#include <Arduino.h>

// ---------------- Pins (edit ONLY here) ----------------
const int R_IN1 = 25;
const int R_IN2 = 26;
const int R_EN  = 27;   // set to 2 if the ENA wire was NOT moved
const int L_IN1 = 32;
const int L_IN2 = 33;
const int L_EN  = 18;   // set to 15 if the ENB wire was NOT moved

// ---------------- Direction fix ----------------
// Change to true ONLY if that wheel turns backward when the code says "forward".
// Do NOT also swap the motor wires, or the two fixes cancel each other.
const bool INVERT_RIGHT = false;
const bool INVERT_LEFT  = false;

// ---------------- PWM and speed limits ----------------
const int PWM_FREQ     = 5000;  // Hz
const int PWM_BITS     = 8;     // duty range 0..255
const int PWM_MAX      = 153;   // 60 % - hard limit (motors rated 3.7 V)
const int PWM_START    = 60;    // ramp start (about 23 %)
const int PWM_STEP     = 10;
const int PWM_TEST     = 110;   // about 43 %
const int RAMP_STEP_MS = 600;

// LEDC channels (used by core 2.x only)
const int R_CH = 0;
const int L_CH = 1;

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

// ---------------- Test helpers ----------------
// Returns true if a key was pressed (and stops the motors).
bool abortRequested() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    stopAll();
    Serial.println("ABORTED");
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

void rampTest(bool right) {
  const char* name = right ? "RIGHT" : "LEFT";
  Serial.printf("\n== %s motor: forward ramp ==\n", name);
  Serial.println("Note the duty where the wheel STARTS to turn, and its direction.");

  int d = PWM_START;
  while (true) {
    int duty = min(d, PWM_MAX);
    if (right) setRight(duty); else setLeft(duty);
    Serial.printf("duty = %d  (%d %%)\n", duty, duty * 100 / 255);
    if (waitMs(RAMP_STEP_MS)) return;
    if (duty == PWM_MAX) break;
    d += PWM_STEP;
  }

  stopAll();
  if (waitMs(1000)) return;

  Serial.printf("== %s motor: backward, duty = %d, 1.5 s ==\n", name, PWM_TEST);
  if (right) setRight(-PWM_TEST); else setLeft(-PWM_TEST);
  if (waitMs(1500)) return;

  stopAll();
  Serial.println("Done.");
}

void bothForwardTest() {
  Serial.printf("\n== BOTH forward, duty = %d, 2 s (robot on the floor, clear space ahead) ==\n", PWM_TEST);
  setRight(PWM_TEST);
  setLeft(PWM_TEST);
  if (waitMs(2000)) return;

  stopAll();
  Serial.println("Done. Did it go straight? Note which side it drifted to.");
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

  Serial.println("\nMotor test ready.");
  Serial.printf("PWM_MAX = %d (%d %%), PWM_TEST = %d\n", PWM_MAX, PWM_MAX * 100 / 255, PWM_TEST);
  Serial.println("Commands: r = right ramp | l = left ramp | b = both forward | s = stop");
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  delay(20);                                  // let the rest of the line arrive
  while (Serial.available()) Serial.read();   // drain it, so a newline does not abort the test

  switch (c) {
    case 'r': rampTest(true);   break;
    case 'l': rampTest(false);  break;
    case 'b': bothForwardTest(); break;
    case 's': stopAll(); Serial.println("STOP"); break;
    default: break;                            // ignore anything else
  }
}
