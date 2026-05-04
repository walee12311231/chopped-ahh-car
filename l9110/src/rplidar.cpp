#include "rplidar.h"

namespace rplidar {

static HardwareSerial* s_uart = nullptr;
static uint8_t s_motorPin = 0;
static uint8_t s_pwmCh = 0;

static Sample s_acc[MAX_SAMPLES_PER_REV];
static size_t s_accN = 0;
static bool   s_truncated = false;

static Stats s_stats{};

static uint8_t s_win[5];
static uint8_t s_winFill = 0;

void begin(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin,
           uint8_t motorPin, uint8_t pwmChannel) {
  s_uart = &uart;
  s_motorPin = motorPin;
  s_pwmCh = pwmChannel;

  s_uart->begin(115200, SERIAL_8N1, rxPin, txPin);
  s_uart->setRxBufferSize(1024);

  ledcSetup(s_pwmCh, 5000, 8);
  ledcAttachPin(s_motorPin, s_pwmCh);
  ledcWrite(s_pwmCh, 0);
}

void setMotor(bool on) { ledcWrite(s_pwmCh, on ? 255 : 0); }

void startScan() {
  uint8_t cmd[2] = {0xA5, 0x20};
  s_uart->write(cmd, 2);
  uint32_t t0 = millis();
  size_t got = 0;
  uint8_t desc[7];
  while (got < 7 && millis() - t0 < 200) {
    if (s_uart->available()) desc[got++] = s_uart->read();
  }
  s_accN = 0;
  s_winFill = 0;
}

void stopScan() {
  uint8_t cmd[2] = {0xA5, 0x25};
  s_uart->write(cmd, 2);
  delay(5);
  while (s_uart->available()) s_uart->read();
  s_winFill = 0;
}

// b0: bits[7..2]=quality, bit1=!S, bit0=S  =>  S xor !S == 1
// b1: bits[7..1]=angle_low7, bit0=C        =>  C == 1
// angle_q6 = ((b1>>1) | (b2<<7)) -> 14-bit, /64 -> degrees
// distance: (b3 | b4<<8) Q14.2; mm = raw>>2
static bool decode(const uint8_t* b, Sample& out) {
  uint8_t S    = b[0] & 0x01;
  uint8_t notS = (b[0] >> 1) & 0x01;
  uint8_t C    = b[1] & 0x01;
  if ((S ^ notS) != 1) return false;
  if (C != 1) return false;

  uint8_t  quality  = b[0] >> 2;
  uint16_t angle_q6 = ((uint16_t)b[1] >> 1) | ((uint16_t)b[2] << 7);
  uint16_t dist_q2  = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
  uint16_t dist_mm  = dist_q2 >> 2;

  if (dist_mm > 12000) dist_mm = 0;
  if (quality == 0)    dist_mm = 0;
  out.angle_q6 = angle_q6;
  out.dist_mm  = dist_mm;
  out.quality  = quality;
  return true;
}

bool poll(Sample* out_buf, size_t out_cap, size_t& out_n, bool& truncated) {
  if (!s_uart) return false;
  bool full_rev = false;

  while (s_uart->available()) {
    if (s_winFill < 5) {
      s_win[s_winFill++] = s_uart->read();
      if (s_winFill < 5) continue;
    }

    Sample sm;
    if (decode(s_win, sm)) {
      s_stats.samples_total++;

      bool start = (s_win[0] & 0x01) != 0;
      if (start && s_accN > 0) {
        size_t n = s_accN < out_cap ? s_accN : out_cap;
        memcpy(out_buf, s_acc, n * sizeof(Sample));
        out_n = n;
        truncated = s_truncated;
        s_accN = 0;
        s_truncated = false;
        s_stats.revolutions++;
        full_rev = true;
      }

      if (s_accN < MAX_SAMPLES_PER_REV) {
        s_acc[s_accN++] = sm;
      } else {
        s_truncated = true;
      }

      s_winFill = 0;
    } else {
      s_stats.samples_bad++;
      memmove(s_win, s_win + 1, 4);
      s_winFill = 4;
    }

    if (full_rev) break;
  }

  return full_rev;
}

const Stats& stats() { return s_stats; }

} // namespace rplidar
