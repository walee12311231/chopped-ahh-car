#pragma once
#include <Arduino.h>
#include "rplidar.h"

namespace packet {

// Scan frame (type 0x01): unchanged on-the-wire format. See README/CLAUDE.md.
size_t buildScan(uint8_t* out, size_t out_cap,
                 uint32_t seq, uint32_t t_ms,
                 const rplidar::Sample* s, size_t n,
                 bool truncated);

// IMU frame (type 0x02). Same 16-byte header layout as a scan frame, with:
//   type=0x02, flags=0x01, n=1, plen=10
// Payload (10 B, LE): yaw_cdeg(i16) ax(i16) ay(i16) az(i16) reserved(u16=0)
// Trailer: 1-byte XOR over all preceding bytes. Total = 27 bytes.
constexpr size_t IMU_FRAME_BYTES = 16 + 10 + 1;

size_t buildImu(uint8_t* out, size_t out_cap,
                uint32_t seq, uint32_t t_ms,
                int16_t yaw_cdeg, int16_t ax, int16_t ay, int16_t az);

} // namespace packet
