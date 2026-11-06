/*
  test_speaker.ino - MAX98357A I2S amplifier + speaker

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  HOW TO TEST
    1. Wire: VIN->5V (from the Buck), GND->GND, BCLK->GPIO23, LRC->GPIO16, DIN->GPIO17.
       Speaker (4-8 ohm) to the + and - output terminals. Leave GAIN and SD unconnected.
       Never connect a speaker terminal to GND.
    2. Upload, open Serial Monitor at 115200.
    3. Expected: every 3 seconds three rising beeps (440, 660, 880 Hz), clean and not distorted.
    4. Silent: check BCLK/LRC/DIN wires and the 5V supply.
       Distorted: lower VOLUME. Hiss or clicks: check that GND is common and keep wires short.
    5. Start with a low VOLUME. A small speaker can be damaged by loud, long tones.
*/

#include <Arduino.h>
#include <driver/i2s.h>

const int PIN_I2S_BCLK = 23;
const int PIN_I2S_LRC  = 16;
const int PIN_I2S_DIN  = 17;

const i2s_port_t SPK_PORT = I2S_NUM_1;       // the microphone uses I2S_NUM_0
const int SAMPLE_RATE = 16000;
const int VOLUME = 3000;                     // peak amplitude, 0..32767 (start low)

void spkInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // stereo, the amp mixes L+R to mono
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_LRC;
  pins.data_out_num = PIN_I2S_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(SPK_PORT, &cfg, 0, NULL);
  i2s_set_pin(SPK_PORT, &pins);
  i2s_zero_dma_buffer(SPK_PORT);
}

// Plays a sine tone with a short fade in/out (no clicks).
void playTone(int freq, int ms) {
  const int FRAMES = 128;
  int16_t buf[FRAMES * 2];
  int total = SAMPLE_RATE * ms / 1000;
  int fade = SAMPLE_RATE / 100;                  // 10 ms
  float phase = 0;
  float step = 2.0f * PI * freq / SAMPLE_RATE;

  for (int done = 0; done < total; done += FRAMES) {
    int n = min(FRAMES, total - done);
    for (int i = 0; i < n; i++) {
      int pos = done + i;
      float env = 1.0f;
      if (pos < fade) env = (float)pos / fade;
      if (total - pos < fade) env = (float)(total - pos) / fade;
      int16_t s = (int16_t)(VOLUME * env * sinf(phase));
      buf[2 * i] = s;
      buf[2 * i + 1] = s;
      phase += step;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    size_t bw = 0;
    i2s_write(SPK_PORT, buf, n * 2 * sizeof(int16_t), &bw, portMAX_DELAY);
  }
}

void silence(int ms) {
  int16_t buf[256] = {};
  int total = SAMPLE_RATE * ms / 1000;
  for (int done = 0; done < total; done += 128) {
    size_t bw = 0;
    i2s_write(SPK_PORT, buf, sizeof(buf), &bw, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  spkInit();
  Serial.println("Speaker test: three beeps every 3 seconds.");
}

void loop() {
  Serial.println("beep 440 Hz");
  playTone(440, 300);
  Serial.println("beep 660 Hz");
  playTone(660, 300);
  Serial.println("beep 880 Hz");
  playTone(880, 300);
  silence(2000);
}
