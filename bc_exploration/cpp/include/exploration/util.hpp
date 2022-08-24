#pragma once
#include <cmath>
#include <iostream>
#include <limits>

inline int
index2dTo1d(const int &rowIdx, const int &colIdx, const int &cols) {
    return rowIdx * cols + colIdx;
}

inline void
index1dTo2d(int idx, const int &cols, int &rowIdx, int &colIdx) {
    rowIdx = idx / cols;
    colIdx = idx % cols;
}

inline float
euclidean(const int &x1, const int &y1, const int &x2, const int &y2) {
    int dx = x2 - x1;
    int dy = y2 - y1;
    return (float) std::sqrt(dx * dx + dy * dy);
}


inline void
clipRange(const int *minRange, const int *maxRange, const int *mapShape, int *clippedMinRange, int *clippedMaxRange) {
    clippedMinRange[0] = std::max(minRange[0], 0);
    clippedMinRange[1] = std::max(minRange[1], 0);
    clippedMaxRange[0] = std::min(maxRange[0], mapShape[0]);
    clippedMaxRange[1] = std::min(maxRange[1], mapShape[1]);
}

inline int
getAngleIdx(const float angle, const float *angles, const int num_angles) {
    float angularDistance = std::numeric_limits<float>::infinity();

    int angleIdx = -1;
    for (int i = 0; i < num_angles; i++) {
        float d = std::abs(angle - angles[i]);
        if (d < angularDistance) {
            angleIdx = i;
            angularDistance = d;
        }
    }

    return angleIdx;
}

inline void
printCoord(const int *coord) {
    std::cout << "[" << coord[0] << ", " << coord[1] << "]" << std::endl;
}
