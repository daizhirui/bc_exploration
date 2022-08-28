#include "exploration/collision.hpp"
#include "exploration/util.hpp"

namespace exploration {
    bool
    checkForCollision(
        const Eigen::Ref<const Eigen::Vector2i> &position,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const Eigen::Ref<const Eigen::MatrixX<bool>> &footprintMask,
        const Eigen::Ref<const Eigen::MatrixXi> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues) {

        int mapShape[2] = {(int)occupancyMap.rows(), (int)occupancyMap.cols()};

        // check if footprint is out of bounds -- if so, it is a collision
        // if in bounds, check if the outline coord is colliding
        for (int i = 0; i < outlineCoords.rows(); ++i) {
            int row = outlineCoords.coeffRef(i, 0) + position.coeffRef(0);
            int col = outlineCoords.coeffRef(i, 1) + position.coeffRef(1);
            if (row < 0 || row >= mapShape[0]) { return true; }

            if (col < 0 || col >= mapShape[1]) { return true; }

            for (int k = 0; k < obstacleValues.size(); k++) {
                if (occupancyMap.coeffRef(row, col) == obstacleValues.coeffRef(k)) { return true; }
            }
        }

        int maskRadius = ((int)footprintMask.rows() - 1) / 2;
        int clippedMinRange[2], clippedMaxRange[2];
        const int minRange[2] = {position.coeffRef(0) - maskRadius, position.coeffRef(1) - maskRadius};
        const int maxRange[2] = {position.coeffRef(0) + maskRadius, position.coeffRef(1) + maskRadius};
        clipRange(minRange, maxRange, mapShape, clippedMinRange, clippedMaxRange);

        for (int m = clippedMinRange[0] - minRange[0], i = clippedMinRange[0]; i < clippedMaxRange[0] + 1; m++, i++) {
            for (int n = clippedMinRange[1] - minRange[1], j = clippedMinRange[1]; j < clippedMaxRange[1] + 1; n++, j++) {
                if (footprintMask.coeffRef(m, n)) {
                    for (int k = 0; k < obstacleValues.size(); k++) {
                        if (occupancyMap.coeffRef(i, j) == obstacleValues.coeffRef(k)) { return true; }
                    }
                }
            }
        }
        return false;
    }
}
