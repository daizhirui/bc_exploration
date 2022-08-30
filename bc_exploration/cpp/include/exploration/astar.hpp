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

    inline std::vector<float>
    getAstarAngles() {
        // these angles correspond to the x,y world converted
        // angles of moving in the corresponding
        // children direction in the astar algorithm
        // see astar assignment of children.
        return {-M_PI, -3. * M_PI_4, -M_PI_2, -M_PI_4, 0., M_PI_4, M_PI_2, 3. * M_PI_4};
    }

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

    std::vector<std::pair<bool, Eigen::MatrixX3f>>
    orientedAstarPrioritizedMultiGoals(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::MatrixX2i> &goals,
        const Eigen::Ref<const Eigen::VectorXf> &goalInitPriorities,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks,
        const Eigen::Ref<const Eigen::VectorXf> &maskAngles,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal);
}  // namespace exploration
