#include "SlamMap.h"
#include <string.h>

void SlamMap::init() {
    const int N = SLAM_MAP_SIZE * SLAM_MAP_SIZE;
    for (int i = 0; i < N; i++) cells[i] = SLAM_CELL_UNKNOWN;
}
