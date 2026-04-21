  An RC car using TinySLAM on an ESP32 + TF-Luna LiDAR to drive around,
  scan its surroundings, and stream the occupancy grid to a computer
  over TCP for visualization and BFS/DFS/A* maze solving.
   ## Hardware
  - ESP32 dev board
  - TF-Luna ToF LiDAR (UART on GPIO 16/17)
  - PCA9685 servo driver (I²C on GPIO 21/22) + 1 hobby servo
  - Motor driver + chassis (TBD — stubs in place)

  ## Build & flash
  pio run --target upload && pio device monitor

  ## Visualizer
  pip install pygame numpy
  python viz/maze_viz.py   # ESP32 AP: SLAM_CAR / slamcar123 @ 192.168.4.1:8080

  ## Controls
  - W A S D / X       drive / stop   (motor stubs for now)
  - ← / →             step servo ±5° and capture a TF-Luna beam
  - C                 commit buffered beams to SLAM
  - E                 export 128×128 occupancy grid
  - click + SPACE     set start/end, solve with BFS / DFS / A*

  ## How it works
  Per scan: random-search localisation picks the best pose from 200
  candidates, then Bresenham ray-cast updates the occupancy grid
  (32 KB @ 16 mm/cell, covers ~2 m — enough for a 36" maze).

  Based on Steux & El Hamzaoui, *TinySLAM* (ICARCV 2010).

  ## Layout
  - src/main.cpp         firmware (WiFi AP + TCP protocol)
  - lib/slam/            TinySLAM, SlamMap, TFLuna, LidarScan
  - viz/maze_viz.py      pygame client + pathfinders
  - CLAUDE.md            full wiring / protocol / tuning reference
