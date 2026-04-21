#pragma once
#include <math.h>
#include <stdint.h>

#ifndef SLAM_MAX_BEAMS
#define SLAM_MAX_BEAMS 360
#endif

struct LidarScan {
    float angles[SLAM_MAX_BEAMS];     // beam angle in radians (robot frame)
    float distances[SLAM_MAX_BEAMS];  // range in metres; NaN = invalid
    int   num_beams;

    void reset() {
        num_beams = 0;
        for (int i = 0; i < SLAM_MAX_BEAMS; i++) distances[i] = NAN;
    }

    // Silently drops beams beyond SLAM_MAX_BEAMS.
    void addBeam(float angle_rad, float dist_m) {
        if (num_beams < SLAM_MAX_BEAMS) {
            angles[num_beams]    = angle_rad;
            distances[num_beams] = dist_m;
            num_beams++;
        }
    }
};

struct Pose {
    float x;      // metres, positive = East
    float y;      // metres, positive = North
    float theta;  // radians, positive = counter-clockwise from East
};
