# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PlatformIO project targeting the **ESP32** (Arduino framework).  
Implements **TinySLAM** — a 2-D occupancy-grid SLAM algorithm — for an ESP32 paired with a **TF-Luna** ToF LiDAR on a **PCA9685** servo sweep, designed to map a **36 × 36 inch** maze.

Reference: Steux & El Hamzaoui, "TinySLAM: A SLAM algorithm in less than 200 lines of C-code", ICARCV 2010.

## Current State

The firmware runs in **WiFi AP + manual-control mode** (`src/main.cpp`).
The ESP32 creates a WiFi access point (`SLAM_CAR` / `slamcar123`) and listens for TCP commands on port 8080. The user steps the TF-Luna servo with arrow keys (one beam per step), commits accumulated beams to SLAM with `c`, and exports the occupancy grid with `e`.

A Python visualizer (`viz/maze_viz.py`) connects over WiFi, sends drive + servo + scan commands, receives the map, overlays a 36×36″ boundary, and runs three pathfinding algorithms (BFS, DFS, A*) for comparison.

**Motor control is not yet implemented** — `moveForward`, `moveBackward`, `turnLeft`, `turnRight`, `stopMotors` are empty stubs in `src/main.cpp` dispatched from the `w/a/s/d/x` TCP commands. Bodies will be filled once motor driver hardware is chosen.

## Build & Flash Commands

```bash
pio run                                          # compile only
pio run --target upload                          # compile + flash
pio run --target upload && pio device monitor    # flash then open serial monitor
pio device monitor                               # serial monitor only (115200 baud)
pio run --target clean                           # clean build artefacts
pio test                                         # on-device unit tests
```

**If upload gets stuck at `Connecting........_____`:** hold the **BOOT** button on the ESP32, tap **EN** (reset), then release BOOT.

## Python Visualizer

```bash
pip install pygame numpy        # one-time setup
python viz/maze_viz.py          # connect to ESP32 at 192.168.4.1
python viz/maze_viz.py --file slam_map.bin  # load a saved map offline
```

Controls (connected to ESP32):
- `W` / `A` / `S` / `D` — drive (forward / left / backward / right) — empty motor stubs
- `X` — stop motors
- `←` / `→` — step servo −5° / +5° and capture one TF-Luna beam into the scan buffer
- `C` — commit the buffered beams to SLAM (updates the map, clears the buffer)
- `E` — export map from ESP32

Controls (visualization):
- Left-click — set maze start (green)
- Right-click — set maze end (red)
- `SPACE` — run BFS, DFS, A* and show comparison
- `R` — reset start/end points
- `F2` — save map to `slam_map.bin` (moved off `S` to free it for WASD)
- `Q` / `ESC` — quit

The red rectangle overlay on the map shows the 36×36″ physical maze centered on the robot's start pose.

### Scan → solve workflow

1. Press `←` / `→` repeatedly to sweep the servo across the field of view. Each press captures one beam; the ESP32 responds with `#SERVO deg=X.X dist=Y.YYm beams=N`.
2. Press `C` to fold the buffered beams into SLAM. Map cells update; buffer clears. `#SCAN_DONE ...` or `#SCAN_EMPTY` is returned.
3. (Drive to a new position with WASD once motor stubs are filled, then repeat 1–2 to extend the map.)
4. Press `E` to export. The occupancy grid streams over TCP.
5. Left-click a start cell and right-click an end cell; press `SPACE` to run BFS, DFS, and A* and compare them in the stats panel.

## Hardware Wiring

### TF-Luna → ESP32 (UART)

| TF-Luna pin | ESP32 pin     |
|-------------|---------------|
| 5V / VCC    | 5V (VIN)      |
| GND         | GND           |
| TX          | GPIO 16 (RX2) |
| RX          | GPIO 17 (TX2) |

### PCA9685 → ESP32 (I2C)

| PCA9685 pin | ESP32 pin  |
|-------------|------------|
| VCC         | 3.3V       |
| GND         | GND        |
| SDA         | GPIO 21    |
| SCL         | GPIO 22    |
| V+          | 5V (servo power) |

Servo on PCA9685 channel 0.  Tune `SERVO_MIN_PULSE` / `SERVO_MAX_PULSE` in `src/main.cpp` for your specific servo (defaults: 150–600 for 0–180°).

## Map Configuration

Current build flags (`platformio.ini`):

| Parameter | Value | Coverage |
|-----------|-------|----------|
| `SLAM_MAP_SIZE` | `128` | 128 × 128 cells |
| `SLAM_MAP_SCALE` | `0.016f` | 16 mm per cell |
| **Total coverage** | | **2.048 m (~80″)** per side |
| **RAM usage** | | **32 KB** (vs 128 KB at 256×256) |

The map was reduced from 256×256 to 128×128 to fit alongside the WiFi stack in ESP32 DRAM (~320 KB total). 32 KB map + ~50 KB WiFi leaves comfortable headroom.

For a 36″ maze, 16 mm/cell gives ~57 cells across the maze — sufficient to resolve corridors and walls.

## WiFi TCP Protocol

**Commands** (client → ESP32): single ASCII characters

| Char | Action |
|------|--------|
| `w` | `moveForward()` — drive stub (empty) |
| `a` | `turnLeft()` — drive stub (empty) |
| `s` | `moveBackward()` — drive stub (empty) |
| `d` | `turnRight()` — drive stub (empty) |
| `x` | `stopMotors()` — drive stub (empty) |
| `l` | Step servo by `−SERVO_STEP_DEG` (default −5°), capture one TF-Luna beam into `scanBuffer` |
| `r` | Step servo by `+SERVO_STEP_DEG` (default +5°), capture one TF-Luna beam into `scanBuffer` |
| `c` | Commit `scanBuffer` to SLAM via `slam.update(scanBuffer)`, then reset the buffer |
| `e` | Export full occupancy grid as binary |

**Map export format** (ESP32 → client):
```
4 bytes   magic     "SMAP"
2 bytes   uint16    map_size (little-endian)
4 bytes   float     scale
4 bytes   float     pose_x
4 bytes   float     pose_y
4 bytes   float     pose_theta
size*size*2 bytes   uint16 cells (row-major, little-endian)
```

**Status messages** (ESP32 → client): text lines prefixed with `#`
- `#READY` — emitted once when a client connects
- `#SERVO deg=X.X dist=Y.YYm beams=N` — after each `l` / `r` step (dist may be `NA`)
- `#SCAN_DONE beams=N x=... y=... theta=...` — after `c` commits the buffer to SLAM
- `#SCAN_EMPTY` — `c` received with nothing in the buffer
- `#EXPORT_DONE WxH K.KKB` — after a successful `e`

## Library: `lib/slam/`

| File | Responsibility |
|------|----------------|
| `slam.h` | Umbrella include — use `#include "slam.h"` |
| `LidarScan.h` | `Pose` (x, y, θ) and `LidarScan` (angle/distance beam array) |
| `TFLuna.h/.cpp` | TF-Luna driver — UART (Serial2) and I2C modes |
| `SlamMap.h/.cpp` | Occupancy grid (`uint16_t` cells, configurable size) |
| `TinySLAM.h/.cpp` | Core algorithm: random-search localisation + Bresenham ray-cast map updates |

### What TinySLAM does per update call

1. **Odometry prediction** — applies dx/dy/dθ deltas to the current pose (optional; pass zeros if no encoders)
2. **Random-search localisation** — generates `SLAM_SEARCH_ITER` (default 200) perturbed pose candidates near the current estimate; keeps whichever one's scan endpoints best overlap the occupied cells already in the map (lower mean cell value = better match)
3. **Ray-cast map update** — for every valid beam, runs Bresenham from robot position to the endpoint:
   - Cells along the ray are nudged toward `SLAM_CELL_FREE` (65535)
   - The endpoint cell is nudged toward `SLAM_CELL_OCCUPIED` (0)
   - Update strength is controlled by `SLAM_UPDATE_QUALITY`

### Key memory constraint

**Always declare `TinySLAM slam;` as a global variable** — it overflows the default FreeRTOS task stack if declared locally inside a function.

### Cell value convention

| Value | Macro | Meaning |
|-------|-------|---------|
| `0` | `SLAM_CELL_OCCUPIED` | Obstacle |
| `65535` | `SLAM_CELL_FREE` | Free space |
| `32768` | `SLAM_CELL_UNKNOWN` | Not yet observed (initial state) |

### Tuning knobs

Override with `-D` flags in `platformio.ini` `build_flags`, or `#define` before including `slam.h`.

| Macro | Default | Effect |
|-------|---------|--------|
| `SLAM_MAP_SIZE` | `256` | Grid side length in cells |
| `SLAM_MAP_SCALE` | `0.10f` | Metres per cell |
| `SLAM_SEARCH_ITER` | `200` | Candidate poses evaluated per update |
| `SLAM_SIGMA_XY` | `0.05f` | Translational search noise, metres |
| `SLAM_SIGMA_THETA` | `0.02f` | Rotational search noise, radians (~1.1°) |
| `SLAM_UPDATE_QUALITY` | `50` | Map update strength [1–255] |
| `SLAM_MAX_BEAMS` | `360` | Maximum beams stored per `LidarScan` |

### API quick reference

```cpp
// Scan + SLAM update
LidarScan scan;
scan.reset();
scan.addBeam(angle_rad, dist_m);
Pose p = slam.update(scan);                    // no odometry
Pose p = slam.update(scan, dx, dy, dtheta);    // with odometry

// Accessors
slam.init(start_x, start_y, theta);  // reset at a known pose
slam.setSeed(42);                    // reproducible random search
const Pose &p = slam.pose();        // current best-estimate pose
SlamMap    &m = slam.map();          // direct map access

// Map helpers
int   mx = m.worldToMap(world_x);   // metres → cell index
float wx = m.mapToWorld(mx);        // cell index → metres
float p  = m.occupancy(mx, my);     // [0,1]; 0=free, 1=occupied
```

### TFLuna API

```cpp
TFLuna luna(Serial2);
luna.begin(0);                                // pass 0 if Serial2 already opened

luna.read(dist_m)                             // non-blocking
luna.read(dist_m, &strength)                  // + signal amplitude
luna.readBlocking(dist_m, 50)                 // blocks up to 50 ms
luna.lastDistance()                            // last good reading (metres)
```

Valid measurement range: **0.2 m – 8.0 m**.
