#pragma once
#include <Arduino.h>

namespace rplidar {

constexpr size_t MAX_SAMPLES_PER_REV = 720;

struct Sample {
  uint16_t angle_q6;   // degrees * 64, 0..23039
  uint16_t dist_mm;    // 0 = invalid
  uint8_t  quality;    // 0..63
} __attribute__((packed));

struct Stats {
  uint32_t samples_total;
  uint32_t samples_bad;
  uint32_t revolutions;
};

void begin(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin,
           uint8_t motorPin, uint8_t pwmChannel);
void setMotor(bool on);
void startScan();
void stopScan();

bool poll(Sample* out_buf, size_t out_cap, size_t& out_n, bool& truncated);

const Stats& stats();

} // namespace rplidar
