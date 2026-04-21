#pragma once
#include "SlamMap.h"
#include "LidarScan.h"

// Tuning knobs — override with -D flags in platformio.ini.

// Random pose candidates evaluated per update.
// Higher = better localisation, slower update.
#ifndef SLAM_SEARCH_ITER
#define SLAM_SEARCH_ITER 200
#endif

// 1-σ translational noise during random search (metres).
#ifndef SLAM_SIGMA_XY
#define SLAM_SIGMA_XY 0.05f
#endif

// 1-σ rotational noise during random search (radians, ~1.1°).
#ifndef SLAM_SIGMA_THETA
#define SLAM_SIGMA_THETA 0.02f
#endif

// Map update strength per scan [1, 255].
// Low = conservative update; high = faster convergence, outlier-sensitive.
#ifndef SLAM_UPDATE_QUALITY
#define SLAM_UPDATE_QUALITY 50
#endif

// TinySLAM / CoreSLAM:
//   Steux & El Hamzaoui, "TinySLAM: A SLAM algorithm in less than 200 lines
//   of C-code", ICARCV 2010.
//
// Per scan update:
//   1. Apply odometry (optional) to current best pose.
//   2. Random search: generate SLAM_SEARCH_ITER candidate poses near the
//      current estimate; keep the one whose scan endpoints best match the
//      occupied cells already in the map.
//   3. Ray-cast each beam at the accepted pose:
//      • cells along the ray  → nudged toward FREE    (Bresenham)
//      • cell at the endpoint → nudged toward OCCUPIED
//
// IMPORTANT: declare TinySLAM as a *global* variable.  The embedded SlamMap
// is ~128 KB and will overflow a FreeRTOS task stack if declared locally.
class TinySLAM {
public:
    TinySLAM();

    void init(float start_x     = 0.0f,
              float start_y     = 0.0f,
              float start_theta = 0.0f);

    // dx, dy in the robot's local frame; dtheta in radians.
    // Returns the updated best-estimate pose.
    Pose update(const LidarScan &scan,
                float dx = 0.0f, float dy = 0.0f, float dtheta = 0.0f);

    const Pose    &pose() const { return _pose; }
          SlamMap &map()        { return _map;  }
    const SlamMap &map()  const { return _map;  }

    void setSeed(uint32_t seed) { _rng = seed; }

private:
    SlamMap  _map;
    Pose     _pose;
    uint32_t _rng;

    // Mean cell value at scan endpoints for pose p; lower = better match.
    uint32_t scorepose(const LidarScan &scan, const Pose &p) const;

    // Bresenham ray-cast: mark free along ray, occupied at endpoint.
    void castRay(int x0, int y0, int x1, int y1, int quality);

    void updateMap(const LidarScan &scan, const Pose &p);

    uint32_t randU32();
    float    randFloat();   // uniform [0, 1)
    float    randNormal();  // N(0, 1) via Box-Muller
};
