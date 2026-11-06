/*
  mic_capture_stream.ino - capture speech with the INMP441 and send each utterance to the PC over WiFi

  PIN MAP (identical in every test sketch)
    L298N  IN1=25 IN2=26 ENA=27 | IN3=32 IN4=33 ENB=18
    OLED   SDA=21 SCL=22
    INMP441 SCK=4 WS=19 SD=34 (L/R to GND) | MAX98357A BCLK=23 LRC=16 DIN=17
    Servo sensor=14, Servo screen=13 | HC-SR04 TRIG=5 ECHO=35 | PIR OUT=36 | Battery=39

  WHAT IT DOES
    1. Connects to your WiFi (the modem) and to pc_receiver.py running on the PC.
    2. Measures the room noise for 1 second at start-up (stay quiet).
    3. Listens all the time. When someone speaks it sends: 320 ms from BEFORE the speech started,
       then the speech, then stops after 0.8 s of silence (8 s maximum).
    4. The PC saves every utterance as a WAV file (16 kHz, 16 bit, mono).

  PROTOCOL (simple TCP, no extra libraries)
    Each message = 2 bytes length (little endian) + that many bytes of PCM audio.
    A message with length 0 means "end of utterance".

  HOW TO TEST
    1. On the PC: python pc_receiver.py   (allow Python through the Windows firewall on PRIVATE networks).
       Find the PC address with the command ipconfig (IPv4 Address) and write it in PC_IP below.
    2. Fill WIFI_SSID, WIFI_PASS and PC_IP. The ESP32 supports 2.4 GHz only.
    3. Upload, open Serial Monitor at 115200. Expected: "WiFi OK", "Connecting to PC ... OK".
    4. Say a sentence. Expected: "SPEECH START", then "SPEECH END ... (sent)", and a new file in the
       recordings folder on the PC. Play it: it must be clear.
    5. It triggers on noise, or never triggers: change MIN_START_RMS or START_FACTOR (the printed levels help).
    6. LATER: the motors must be stopped, and the speaker silent, while the robot listens.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>

// ===================== 1) NETWORK =====================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* PC_IP     = "192.168.1.10";
const uint16_t PC_PORT = 5005;

// ===================== 2) MICROPHONE =====================
const int PIN_I2S_SCK = 4;
const int PIN_I2S_WS  = 19;
const int PIN_I2S_SD  = 34;
const i2s_port_t MIC_PORT = I2S_NUM_0;
const int  SAMPLE_RATE   = 16000;
const int  FRAME_SAMPLES = 256;          // 16 ms per frame
const int  MIC_SHIFT     = 14;           // lower = louder
const bool MIC_USE_LEFT  = true;

// ===================== 3) SPEECH DETECTION (tune these) =====================
const int   PREROLL_FRAMES     = 20;     // 320 ms kept from before the speech
const float START_FACTOR       = 3.0f;   // start when RMS > noise x this ...
const int   MIN_START_RMS      = 250;    // ... and above this absolute level
const int   START_FRAMES       = 3;      // must stay loud for 48 ms
const float END_FACTOR         = 1.8f;
const int   END_SILENCE_FRAMES = 50;     // 0.8 s of quiet ends the utterance
const int   MAX_UTT_FRAMES     = 500;    // 8 s maximum
const bool  DEBUG_LEVELS       = true;   // print the levels twice a second

// ===================== state =====================
WiFiClient client;
int16_t ring[PREROLL_FRAMES][FRAME_SAMPLES];
int ringHead = 0, ringCount = 0;
float noiseRms = 300.0f;
bool speaking = false;
int loudCount = 0, silenceFrames = 0, uttFrames = 0;
unsigned long lastConnectTry = 0, lastDebug = 0;

static inline int clamp16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int)v;
}

void micInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = MIC_USE_LEFT ? I2S_CHANNEL_FMT_ONLY_LEFT : I2S_CHANNEL_FMT_ONLY_RIGHT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = FRAME_SAMPLES;
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

// Reads one frame (16 ms), removes the DC offset, returns its RMS.
bool readFrame(int16_t* out, float* rmsOut) {
  static int32_t raw[FRAME_SAMPLES];
  size_t br = 0;
  i2s_read(MIC_PORT, raw, sizeof(raw), &br, portMAX_DELAY);
  if (br < sizeof(raw)) return false;

  long long sum = 0;
  for (int i = 0; i < FRAME_SAMPLES; i++) {
    out[i] = (int16_t)clamp16(raw[i] >> MIC_SHIFT);
    sum += out[i];
  }
  int mean = (int)(sum / FRAME_SAMPLES);

  double sq = 0;
  for (int i = 0; i < FRAME_SAMPLES; i++) {
    int v = clamp16((long)out[i] - mean);
    out[i] = (int16_t)v;
    sq += (double)v * v;
  }
  *rmsOut = (float)sqrt(sq / FRAME_SAMPLES);
  return true;
}

// ===================== network helpers =====================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 30000) {
      Serial.println("\nWiFi failed: check the name, the password and that the network is 2.4 GHz. Restarting.");
      ESP.restart();
    }
  }
  WiFi.setSleep(false);                         // lower latency
  Serial.printf("\nWiFi OK, ESP32 address %s\n", WiFi.localIP().toString().c_str());
}

bool ensureConnected() {
  if (client.connected()) return true;
  if (millis() - lastConnectTry < 2000) return false;
  lastConnectTry = millis();
  Serial.printf("Connecting to PC %s:%u ... ", PC_IP, PC_PORT);
  if (client.connect(PC_IP, PC_PORT, 2000)) {
    client.setNoDelay(true);
    Serial.println("OK");
    return true;
  }
  Serial.println("failed (is pc_receiver.py running? Windows firewall? same network? correct PC_IP?)");
  return false;
}

bool sendFrame(const int16_t* samples) {
  static uint8_t pkt[2 + FRAME_SAMPLES * 2];
  uint16_t len = FRAME_SAMPLES * 2;
  pkt[0] = len & 0xFF;
  pkt[1] = len >> 8;
  memcpy(pkt + 2, samples, len);
  return client.write(pkt, sizeof(pkt)) == sizeof(pkt);
}

void sendEnd() {
  uint8_t z[2] = { 0, 0 };
  client.write(z, 2);
}

// ===================== Arduino entry points =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  micInit();
  connectWiFi();

  Serial.println("Measuring the room noise for 1 second: stay quiet...");
  static int16_t f[FRAME_SAMPLES];
  float rms = 0, acc = 0;
  for (int i = 0; i < 10; i++) readFrame(f, &rms);       // let the microphone settle
  int n = 0;
  for (int i = 0; i < 62; i++) {
    if (readFrame(f, &rms)) {
      acc += rms;
      n++;
    }
  }
  if (n > 0) noiseRms = max(20.0f, acc / n);
  Serial.printf("Noise level (RMS): %.0f. Speech starts above about %.0f. Speak now.\n",
                noiseRms, max((float)MIN_START_RMS, noiseRms * START_FACTOR));
}

void loop() {
  static int16_t frame[FRAME_SAMPLES];
  float rms = 0;
  if (!readFrame(frame, &rms)) return;
  bool online = ensureConnected();

  if (!speaking) {
    memcpy(ring[ringHead], frame, sizeof(frame));        // keep the last 320 ms
    ringHead = (ringHead + 1) % PREROLL_FRAMES;
    if (ringCount < PREROLL_FRAMES) ringCount++;

    float startThr = max((float)MIN_START_RMS, noiseRms * START_FACTOR);
    if (rms > startThr) loudCount++; else loudCount = 0;
    if (rms < noiseRms * 2.0f) noiseRms = noiseRms * 0.98f + rms * 0.02f;   // follow slow noise changes

    if (loudCount >= START_FRAMES) {
      speaking = true;
      uttFrames = 0;
      silenceFrames = 0;
      loudCount = 0;
      Serial.println("SPEECH START");
      if (online) {
        int first = (ringHead + PREROLL_FRAMES - ringCount) % PREROLL_FRAMES;
        for (int i = 0; i < ringCount; i++) {
          if (!sendFrame(ring[(first + i) % PREROLL_FRAMES])) {
            client.stop();
            online = false;
            break;
          }
        }
      }
      ringCount = 0;
    }
  } else {
    uttFrames++;
    if (online && !sendFrame(frame)) {
      client.stop();
      online = false;
    }
    float endThr = max(MIN_START_RMS * 0.6f, noiseRms * END_FACTOR);
    if (rms < endThr) silenceFrames++; else silenceFrames = 0;

    if (silenceFrames >= END_SILENCE_FRAMES || uttFrames >= MAX_UTT_FRAMES) {
      if (online) sendEnd();
      Serial.printf("SPEECH END %.1f s (%s)\n", uttFrames * FRAME_SAMPLES / (float)SAMPLE_RATE,
                    online ? "sent" : "NOT sent: no connection");
      speaking = false;
      loudCount = 0;
    }
  }

  if (DEBUG_LEVELS && millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.printf("level %5.0f  noise %5.0f  %s\n", rms, noiseRms, speaking ? "[speaking]" : "");
  }
}
