/*
  test_pir.ino - PIR motion sensor HC-SR501

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire: VCC->5V, GND->GND, OUT->GPIO36 (the OUT pin is the MIDDLE pin; its output is 3.3V, safe for the ESP32).
    2. Upload, open Serial Monitor at 115200.
    3. Wait for the warm-up (60 s). Stay away from the sensor and do not move in front of it.
    4. Move your hand in front of it. Expected: "MOTION detected", and after a few seconds "motion ended".
    5. Two small pots on the sensor: one sets sensitivity, the other the hold time (turn the time to
       minimum while testing). The jumper H = retrigger (stays HIGH while you keep moving).
    6. For quick tests you may set WARMUP_MS = 10000, but expect some false triggers.
*/

#include <Arduino.h>

const int PIN_PIR = 36;                    // input-only pin, no pull-up available (the sensor drives it)
const unsigned long WARMUP_MS = 60000;

bool pirState = false;
unsigned long highStart = 0;
int events = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIR, INPUT);

  Serial.println("PIR test. Warm-up: keep still and do not move in front of the sensor.");
  unsigned long t0 = millis();
  while (millis() - t0 < WARMUP_MS) {
    Serial.printf("warm-up: %lu s left\n", (WARMUP_MS - (millis() - t0)) / 1000);
    delay(5000);
  }
  Serial.println("READY. Move your hand in front of the sensor.");
}

void loop() {
  bool s = digitalRead(PIN_PIR) == HIGH;
  if (s != pirState) {
    delay(30);                                  // simple debounce
    if ((digitalRead(PIN_PIR) == HIGH) == s) {
      pirState = s;
      if (s) {
        events++;
        highStart = millis();
        Serial.printf("[%lu ms] MOTION detected (event #%d)\n", millis(), events);
      } else {
        Serial.printf("[%lu ms] motion ended (output was HIGH for %lu ms)\n", millis(), millis() - highStart);
      }
    }
  }
  delay(10);
}
