/*
  test_hcsr04.ino - ultrasonic distance sensor HC-SR04

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire: VCC->5V, GND->GND, TRIG->GPIO5.
       ECHO outputs 5V and the ESP32 tolerates only 3.3V, so ECHO must go through a voltage divider:
       ECHO --1k--+-- GPIO35      (and 2k from that junction to GND)
    2. Upload, open Serial Monitor at 115200.
    3. Put a flat board (or a book) in front of the sensor at 10, 20, 30 and 50 cm measured with a ruler.
       Expected: the printed value is within about 1 cm of the ruler.
    4. "no echo" printed all the time: check the divider, the 5V supply and TRIG/ECHO wires.
       Soft or angled surfaces (cloth, sponge) also give no echo: that is normal.
*/

#include <Arduino.h>

const int PIN_TRIG = 5;
const int PIN_ECHO = 35;                       // input-only pin, fine for ECHO
const unsigned long ECHO_TIMEOUT_US = 30000;   // 30 ms = about 5 m
const int SAMPLES = 5;                         // median of 5 readings (max 9)

// One reading in cm, or -1 when there is no echo.
float readOnceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0f;
  return us * 0.0343f / 2.0f;
}

// Median of several readings. A timeout counts as "unknown", not as 0 cm.
float readMedianCm() {
  float v[9];
  int m = 0;
  for (int i = 0; i < SAMPLES; i++) {
    float r = readOnceCm();
    if (r > 0) v[m++] = r;
    delay(30);
  }
  if (m < SAMPLES / 2 + 1) return -1.0f;
  for (int i = 1; i < m; i++) {                // insertion sort
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.println("HC-SR04 test. Put a flat object in front of the sensor.");
}

void loop() {
  float d = readMedianCm();
  if (d < 0) {
    Serial.println("no echo (too far, closer than 2 cm, soft/angled surface, or wiring)");
  } else {
    int bar = min(50, (int)(d / 4));
    Serial.printf("%6.1f cm  ", d);
    for (int i = 0; i < bar; i++) Serial.print("#");
    Serial.println();
  }
  delay(100);
}
