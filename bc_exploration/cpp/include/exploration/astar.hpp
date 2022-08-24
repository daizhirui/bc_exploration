#pragma once

#include "collision.hpp"

namespace exploration {
    struct Node {
        float f;
        int idx;

        Node(const float f, const int idx)
            : f(f),
              idx(idx) {}

        bool
        operator>(const Node &other) const;
    };

    std::pair<bool, Eigen::MatrixX2i>
    astar(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::Vector2i> &goal,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal);

    std::vector<float>
    getAstarAngles();

    std::pair<bool, Eigen::MatrixX3f>
    orientedAstar(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::Vector2i> &goal,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks,
        const Eigen::Ref<const Eigen::VectorXf> &maskAngles,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal,
        std::ostream &ostream = std::cout);

    std::vector<std::pair<bool, Eigen::MatrixX3f>>
    orientedAstarMultiGoals(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::MatrixX2i> &goals,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks,
        const Eigen::Ref<const Eigen::VectorXf> &maskAngles,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal,
        bool parallel = false);
}  // namespace exploration
