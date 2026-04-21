#pragma once
#include <stdint.h>

// Override before including this header:
//   #define SLAM_MAP_SIZE  512   // requires PSRAM
//   #define SLAM_MAP_SCALE 0.05f // 5 cm/cell
//
// Default: 256 × 256 cells × 2 bytes = 128 KB (fits in ESP32 internal RAM).
// IMPORTANT: declare SlamMap (or TinySLAM) as a *global* variable — it is too
// large for the default FreeRTOS task stack.

#ifndef SLAM_MAP_SIZE
#define SLAM_MAP_SIZE  256
#endif

#ifndef SLAM_MAP_SCALE
#define SLAM_MAP_SCALE 0.10f   // metres per cell; 256 cells → 25.6 m square map
#endif

// Cell value semantics (uint16_t):
#define SLAM_CELL_OCCUPIED  0u
#define SLAM_CELL_FREE      65535u
#define SLAM_CELL_UNKNOWN   32768u

class SlamMap {
public:
    // Row-major flat storage: index = y * SLAM_MAP_SIZE + x
    uint16_t cells[SLAM_MAP_SIZE * SLAM_MAP_SIZE];

    void init();   // fills every cell with SLAM_CELL_UNKNOWN

    int   worldToMap(float w)  const { return (int)(w / SLAM_MAP_SCALE) + SLAM_MAP_SIZE / 2; }
    float mapToWorld(int   m)  const { return (m  - SLAM_MAP_SIZE / 2) * SLAM_MAP_SCALE; }

    bool inBounds(int x, int y) const {
        return (unsigned)x < SLAM_MAP_SIZE && (unsigned)y < SLAM_MAP_SIZE;
    }

    // Returns SLAM_CELL_UNKNOWN for out-of-bounds coordinates.
    uint16_t get(int x, int y) const {
        return inBounds(x, y) ? cells[y * SLAM_MAP_SIZE + x] : SLAM_CELL_UNKNOWN;
    }

    void set(int x, int y, uint16_t v) {
        if (inBounds(x, y)) cells[y * SLAM_MAP_SIZE + x] = v;
    }

    // Occupancy probability in [0, 1].  0 = free, 1 = occupied.
    float occupancy(int x, int y) const {
        return 1.0f - get(x, y) * (1.0f / 65535.0f);
    }
};
