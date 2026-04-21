#include "TinySLAM.h"
#include <math.h>
#include <stdint.h>

TinySLAM::TinySLAM() : _rng(0xDEADBEEFu) {
    _pose = {0.0f, 0.0f, 0.0f};
}

void TinySLAM::init(float start_x, float start_y, float start_theta) {
    _map.init();
    _pose = {start_x, start_y, start_theta};
}

uint32_t TinySLAM::randU32() {
    _rng = _rng * 1664525u + 1013904223u;
    return _rng;
}

float TinySLAM::randFloat() {
    return (float)(randU32() >> 8) * (1.0f / (float)(1 << 24));
}

float TinySLAM::randNormal() {
    // Clamp u1 away from zero to avoid log(0).
    float u1 = randFloat() + 1e-9f;
    float u2 = randFloat();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

// Sum of map cell values at every valid beam endpoint / beam count.
// Cells near 0 (occupied) score low → a lower return value = a better match.
// Returns UINT32_MAX when no valid beams exist.
uint32_t TinySLAM::scorepose(const LidarScan &scan, const Pose &p) const {
    uint64_t total = 0;
    int      count = 0;

    for (int i = 0; i < scan.num_beams; i++) {
        const float d = scan.distances[i];
        if (!isfinite(d) || d <= 0.0f) continue;

        const float bx = p.x + d * cosf(p.theta + scan.angles[i]);
        const float by = p.y + d * sinf(p.theta + scan.angles[i]);

        total += _map.get(_map.worldToMap(bx), _map.worldToMap(by));
        count++;
    }

    return (count > 0) ? (uint32_t)(total / count) : UINT32_MAX;
}

// Bresenham walk from (x0,y0) to (x1,y1).  Fixed-point EMA update:
//   free     (along ray): cell += (SLAM_CELL_FREE - cell) * quality >> 8
//   occupied (endpoint):  cell  = cell * (256 - quality) >> 8
void TinySLAM::castRay(int x0, int y0, int x1, int y1, int quality) {
    const int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    const int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;

    for (;;) {
        if (_map.inBounds(x, y)) {
            uint16_t &cell = _map.cells[y * SLAM_MAP_SIZE + x];

            if (x == x1 && y == y1) {
                // Endpoint: nudge toward OCCUPIED (0).
                cell = (uint16_t)((cell * (256 - quality)) >> 8);
            } else {
                // Along ray: nudge toward FREE (65535).
                cell = (uint16_t)(cell + (((SLAM_CELL_FREE - cell) * quality) >> 8));
            }
        }

        if (x == x1 && y == y1) break;

        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

void TinySLAM::updateMap(const LidarScan &scan, const Pose &p) {
    const int rx = _map.worldToMap(p.x);
    const int ry = _map.worldToMap(p.y);

    for (int i = 0; i < scan.num_beams; i++) {
        const float d = scan.distances[i];
        if (!isfinite(d) || d <= 0.0f) continue;

        const float bx = p.x + d * cosf(p.theta + scan.angles[i]);
        const float by = p.y + d * sinf(p.theta + scan.angles[i]);

        castRay(rx, ry, _map.worldToMap(bx), _map.worldToMap(by),
                SLAM_UPDATE_QUALITY);
    }
}

Pose TinySLAM::update(const LidarScan &scan,
                       float dx, float dy, float dtheta) {
    // 1. Odometry prediction (rotate local delta into world frame).
    {
        const float ct = cosf(_pose.theta);
        const float st = sinf(_pose.theta);
        _pose.x     += dx * ct - dy * st;
        _pose.y     += dx * st + dy * ct;
        _pose.theta += dtheta;
    }

    // 2. Random-search localisation (TinySLAM §3.2): keep the best of
    //    SLAM_SEARCH_ITER candidates around the current pose.
    uint32_t bestScore = scorepose(scan, _pose);

    for (int iter = 0; iter < SLAM_SEARCH_ITER; iter++) {
        Pose candidate = _pose;
        candidate.x     += randNormal() * SLAM_SIGMA_XY;
        candidate.y     += randNormal() * SLAM_SIGMA_XY;
        candidate.theta += randNormal() * SLAM_SIGMA_THETA;

        const uint32_t score = scorepose(scan, candidate);
        if (score < bestScore) {
            bestScore = score;
            _pose     = candidate;
        }
    }

    // 3. Update map at the accepted pose.
    updateMap(scan, _pose);

    return _pose;
}
