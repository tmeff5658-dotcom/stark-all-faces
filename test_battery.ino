/*
  test_battery.ino - battery voltage through a resistor divider (2 x 18650 in series, 2S)

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  THE DIVIDER (use 1 % resistors if you can)
        battery +  ---[ R1 = 100 kOhm ]---+---[ R2 = 39 kOhm ]--- GND
                                          |
                                          +--- GPIO39
                                          +--- 100 nF capacitor to GND (smooths the reading)
    At 8.4 V (full) the pin sees 2.36 V. At 6.0 V it sees 1.68 V.
    The ESP32 ADC reads reliably up to about 2.45 V, and a pin must NEVER see more than 3.3 V.

  HOW TO TEST
    1. FIRST check the divider with a multimeter and a 9 V or the battery: the middle point must be
       below 2.5 V. Only then connect it to GPIO39. Take the battery AFTER the BMS and the switch.
    2. Upload, open Serial Monitor at 115200.
    3. Compare the printed battery voltage with a multimeter on the battery terminals.
    4. If they differ, set CAL = (multimeter reading) / (printed reading), upload again.
    5. Percent is only an estimate, and the voltage sags while the motors run.
*/

#include <Arduino.h>

const int PIN_BATT = 39;                // ADC1 channel: works while WiFi is on
const float R1 = 100000.0f;             // ohms, battery+ to the pin
const float R2 = 39000.0f;              // ohms, pin to GND
const float CAL = 1.000f;               // calibration factor (see step 4)

const float V_FULL = 8.4f;              // 2S fully charged
const float V_LOW  = 6.8f;              // show the low-battery face below this
const float V_CRIT = 6.4f;              // stop the motors below this (the BMS cuts near 6.0 V)

float readBatteryV(int *mvAtPin) {
  long mv = 0;
  for (int i = 0; i < 32; i++) {
    mv += analogReadMilliVolts(PIN_BATT);
    delay(2);
  }
  mv /= 32;
  if (mvAtPin) *mvAtPin = (int)mv;
  return (mv / 1000.0f) * (R1 + R2) / R2 * CAL;
}

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
  Serial.println("Battery test. Compare with a multimeter.");
}

void loop() {
  int mv = 0;
  float v = readBatteryV(&mv);
  int pct = constrain((int)lroundf((v - V_CRIT) / (V_FULL - V_CRIT) * 100.0f), 0, 100);
  const char* state = (v <= V_CRIT) ? "CRITICAL" : (v <= V_LOW ? "LOW" : "OK");
  Serial.printf("pin %4d mV  ->  battery %.2f V  (about %3d %%)  %s\n", mv, v, pct, state);
  delay(500);
}
