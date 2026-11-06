/*
  test_oled.ino - OLED check (SSD1306 128x64, I2C)

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire: SDA->GPIO21, SCL->GPIO22, VCC->3.3V, GND->GND.
    2. Upload, open Serial Monitor at 115200.
    3. Expected: "device at 0x3C" and the screen shows "OLED OK" with a counter and a moving box.
    4. Nothing found: swap SDA/SCL, check 3.3V and GND, and check the solder joints.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
unsigned long frames = 0;

void scanI2C() {
  Serial.println("I2C scan...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X\n", a);
      found++;
    }
  }
  if (found == 0) Serial.println("  NOTHING found: check SDA/SCL wires, VCC = 3.3V and GND");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(PIN_SDA, PIN_SCL);
  scanI2C();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed (address 0x3C?). Stopping.");
    for (;;) delay(1000);
  }
  Serial.println("OLED initialised.");
}

void loop() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 4);
  display.print("OLED OK");
  display.setTextSize(1);
  display.setCursor(10, 28);
  display.print("frame: ");
  display.print(frames);
  int x = (int)((frames * 3) % 100);
  display.drawRect(10, 44, 108, 14, SSD1306_WHITE);
  display.fillRect(12 + x, 46, 8, 10, SSD1306_WHITE);
  display.display();
  frames++;
  delay(30);
}
