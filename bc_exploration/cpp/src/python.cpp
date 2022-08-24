#include "exploration/astar.hpp"
#include "pybind11.h"

PYBIND11_MODULE(_exploration_cpp, m) {
    m.def(
         "c_check_for_collision",
         &exploration::checkForCollision,
         py::arg("position"),
         py::arg("occupancyMap"),
         py::arg("footprintMask"),
         py::arg("outlineCoords"),
         py::arg("obstacleValues"))
        .def(
            "c_astar",
            &exploration::astar,
            py::arg("start"),
            py::arg("goal"),
            py::arg("occupancy_map"),
            py::arg("obstacle_values"),
            py::arg("delta"),
            py::arg("epsilon"),
            py::arg("planning_scale"),
            py::arg("allow_diagonal"))
        .def("c_get_astar_angles", &exploration::getAstarAngles)
        .def(
            "c_oriented_astar",
            [](const Eigen::Ref<const Eigen::Vector2i> &start,
               const Eigen::Ref<const Eigen::Vector2i> &goal,
               const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
               const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks,
               const Eigen::Ref<const Eigen::VectorXf> &maskAngles,
               const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords,
               const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
               float delta,
               float epsilon,
               int planningScale,
               bool allowDiagonal) {
                return exploration::orientedAstar(
                    start,
                    goal,
                    occupancyMap,
                    footprintMasks,
                    maskAngles,
                    outlineCoords,
                    obstacleValues,
                    delta,
                    epsilon,
                    planningScale,
                    allowDiagonal);
            },
            py::arg("start"),
            py::arg("goal"),
            py::arg("occupancy_map"),
            py::arg("footprint_masks"),
            py::arg("mask_angles"),
            py::arg("outline_coords"),
            py::arg("obstacle_values"),
            py::arg("delta"),
            py::arg("epsilon"),
            py::arg("planning_scale"),
            py::arg("allow_diagonal"))
        .def(
            "c_oriented_astar_multi_goals",
            &exploration::orientedAstarMultiGoals,
            py::arg("start"),
            py::arg("goal"),
            py::arg("occupancy_map"),
            py::arg("footprint_masks"),
            py::arg("mask_angles"),
            py::arg("outline_coords"),
            py::arg("obstacle_values"),
            py::arg("delta"),
            py::arg("epsilon"),
            py::arg("planning_scale"),
            py::arg("allow_diagonal"),
            py::arg("parallel") = false);
}
