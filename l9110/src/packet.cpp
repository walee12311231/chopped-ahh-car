#include "packet.h"

namespace packet {

size_t buildScan(uint8_t* out, size_t out_cap,
                 uint32_t seq, uint32_t t_ms,
                 const rplidar::Sample* s, size_t n,
                 bool truncated) {
  size_t need = 16 + 5 * n + 1;
  if (need > out_cap) return 0;

  out[0] = 0xA5; out[1] = 0x5A;
  out[2] = 0x01;
  out[3] = 0x01 | (truncated ? 0x02 : 0x00);
  memcpy(out + 4,  &seq,  4);
  memcpy(out + 8,  &t_ms, 4);
  uint16_t n16  = (uint16_t)n;
  uint16_t plen = (uint16_t)(n * 5);
  memcpy(out + 12, &n16,  2);
  memcpy(out + 14, &plen, 2);

  uint8_t* p = out + 16;
  for (size_t i = 0; i < n; ++i) {
    memcpy(p,     &s[i].angle_q6, 2);
    memcpy(p + 2, &s[i].dist_mm,  2);
    p[4] = s[i].quality;
    p += 5;
  }

  uint8_t cs = 0;
  for (size_t i = 0; i < need - 1; ++i) cs ^= out[i];
  out[need - 1] = cs;
  return need;
}

size_t buildImu(uint8_t* out, size_t out_cap,
                uint32_t seq, uint32_t t_ms,
                int16_t yaw_cdeg, int16_t ax, int16_t ay, int16_t az) {
  const size_t need = IMU_FRAME_BYTES; // 27
  if (need > out_cap) return 0;

  out[0] = 0xA5; out[1] = 0x5A;
  out[2] = 0x02;            // IMU frame
  out[3] = 0x01;            // bit0 always set; bit1 unused for IMU
  memcpy(out + 4,  &seq,  4);
  memcpy(out + 8,  &t_ms, 4);
  uint16_t n16  = 1;
  uint16_t plen = 10;       // header n*5 invariant does NOT apply to IMU
  memcpy(out + 12, &n16,  2);
  memcpy(out + 14, &plen, 2);

  uint16_t reserved = 0;
  memcpy(out + 16, &yaw_cdeg, 2);
  memcpy(out + 18, &ax,       2);
  memcpy(out + 20, &ay,       2);
  memcpy(out + 22, &az,       2);
  memcpy(out + 24, &reserved, 2);

  uint8_t cs = 0;
  for (size_t i = 0; i < need - 1; ++i) cs ^= out[i];
  out[need - 1] = cs;
  return need;
}

} // namespace packet
