/*
  test_mic.ino - INMP441 I2S microphone level meter

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire: VDD->3.3V ONLY (5V destroys it), GND->GND, SCK->GPIO4, WS->GPIO19, SD->GPIO34, L/R->GND.
    2. Upload, open Serial Monitor at 115200.
    3. Stay quiet: expected a low, steady RMS (the "silence level", note it).
       Then talk or clap: the RMS and the # bar must jump up.
    4. RMS always 0: check SD/WS/SCK wires, then set MIC_USE_LEFT = false (some boards read the other slot).
       RMS always very high or noisy: keep the mic away from the motors and wires; check GND.
    5. Too quiet: lower MIC_SHIFT (13, 12). Clipping (peak 32767): raise MIC_SHIFT (15).
*/

#include <Arduino.h>
#include <driver/i2s.h>

const int PIN_I2S_SCK = 4;
const int PIN_I2S_WS  = 19;
const int PIN_I2S_SD  = 34;

const i2s_port_t MIC_PORT = I2S_NUM_0;
const int  SAMPLE_RATE  = 16000;
const int  MIC_SHIFT    = 14;       // 32-bit sample -> 16-bit. Lower = louder.
const bool MIC_USE_LEFT = true;     // L/R pin to GND = left slot
const int  BLOCK        = 256;      // samples per read (16 ms)

void micInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = MIC_USE_LEFT ? I2S_CHANNEL_FMT_ONLY_LEFT : I2S_CHANNEL_FMT_ONLY_RIGHT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = BLOCK;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_SCK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_SD;

  i2s_driver_install(MIC_PORT, &cfg, 0, NULL);
  i2s_set_pin(MIC_PORT, &pins);
  i2s_zero_dma_buffer(MIC_PORT);
}

static inline int clamp16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int)v;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  micInit();
  Serial.println("Microphone test. Stay quiet, then talk or clap.");
}

void loop() {
  static int32_t raw[BLOCK];
  static float maxRms = 0;
  static int maxPeak = 0;
  static int blocks = 0;

  size_t br = 0;
  i2s_read(MIC_PORT, raw, sizeof(raw), &br, portMAX_DELAY);
  int n = (int)(br / 4);
  if (n <= 0) return;

  long long sum = 0;
  for (int i = 0; i < n; i++) sum += clamp16(raw[i] >> MIC_SHIFT);
  int mean = (int)(sum / n);

  double sq = 0;
  int peak = 0;
  for (int i = 0; i < n; i++) {
    int v = clamp16((long)clamp16(raw[i] >> MIC_SHIFT) - mean);
    sq += (double)v * v;
    if (abs(v) > peak) peak = abs(v);
  }
  float rms = (float)sqrt(sq / n);
  if (rms > maxRms) maxRms = rms;
  if (peak > maxPeak) maxPeak = peak;

  if (++blocks >= 6) {                       // print about every 100 ms
    int bar = min(50, (int)(maxRms / 80.0f));
    Serial.printf("RMS %6.0f  peak %5d  |", maxRms, maxPeak);
    for (int i = 0; i < bar; i++) Serial.print("#");
    Serial.println();
    maxRms = 0;
    maxPeak = 0;
    blocks = 0;
  }
}
