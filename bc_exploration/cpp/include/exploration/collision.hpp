#pragma once

#include "../pybind11.h"
#include <Eigen/Dense>

namespace exploration {
    bool
    checkForCollision(
        const Eigen::Ref<const Eigen::Vector2i> &position,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const Eigen::Ref<const Eigen::MatrixX<bool>> &footprintMask,
        const Eigen::Ref<const Eigen::MatrixXi> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues);
}
