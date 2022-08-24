#include "exploration/astar.hpp"
#include "exploration/util.hpp"
#include <queue>
#include <stack>


namespace exploration {
    bool
    Node::operator>(const exploration::Node &other) const {
        return f > other.f;
    }

    std::pair<bool, Eigen::MatrixX2i>
    astar(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::Vector2i> &goal,
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal) {

        std::cout << "start plan, ";

        // verify planning scale is correct
        if (planningScale < 1) { throw std::logic_error("ERROR: parameter planningScale of c++ function oriented_astar() must be greater than 1 "); }

        const float inf = std::numeric_limits<float>::infinity();

        int mapShape[2] = {(int) occupancyMap.rows(), (int) occupancyMap.cols()};
        int numCells = (int) occupancyMap.size();
        const int startIdx = index2dTo1d(start.coeffRef(0), start.coeffRef(1), mapShape[1]);

        for (int i = 0; i < obstacleValues.size(); i++) {
            if (occupancyMap.coeffRef(start.coeffRef(0), start.coeffRef(1)) == obstacleValues.coeffRef(i) ||
                occupancyMap.coeffRef(goal.coeffRef(0), goal.coeffRef(1)) == obstacleValues.coeffRef(i)) {
                return std::make_pair(false, Eigen::MatrixXi(0, 2));
            }
        }

        Node startNode = Node(0.0, startIdx);

        std::priority_queue<Node, std::vector<Node>, std::greater<>> openSet;
        openSet.push(startNode);

        std::vector<float> costs(numCells, inf);
        costs[startIdx] = 0.0;

        std::vector<int> paths(numCells, -1);

        int children[8][2];
        int parentCoord[2];
        int solutionIdx = -1;

        int numNodesExpanded = 0;
        bool isSuccessful = false;
        while (!openSet.empty()) {
            Node parent = openSet.top();
            openSet.pop();

            index1dTo2d(parent.idx, mapShape[1], parentCoord[0], parentCoord[1]);

            // todo planningScale + delta is not quite right, maybe max(planningScale, delta) is correct
            float distance_to_goal = euclidean(parentCoord[0], parentCoord[1], goal.x(), goal.y());
            if (distance_to_goal <= delta || (planningScale != 1 && distance_to_goal <= float(planningScale) + delta)) {
                isSuccessful = true;
                solutionIdx = parent.idx;
                break;
            }

            children[0][0] = parentCoord[0];
            children[0][1] = parentCoord[1] - planningScale;
            children[1][0] = parentCoord[0] + planningScale;
            children[1][1] = parentCoord[1] - planningScale;
            children[2][0] = parentCoord[0] + planningScale;
            children[2][1] = parentCoord[1];
            children[3][0] = parentCoord[0] + planningScale;
            children[3][1] = parentCoord[1] + planningScale;
            children[4][0] = parentCoord[0];
            children[4][1] = parentCoord[1] + planningScale;
            children[5][0] = parentCoord[0] - planningScale;
            children[5][1] = parentCoord[1] + planningScale;
            children[6][0] = parentCoord[0] - planningScale;
            children[6][1] = parentCoord[1];
            children[7][0] = parentCoord[0] - planningScale;
            children[7][1] = parentCoord[1] - planningScale;

            for (int c = 0; c < 8; c++) {
                // if diagonal is not allowed, skip those children
                if (!allowDiagonal && c % 2 != 0) { continue; }

                // skip child if out of bounds
                if (children[c][0] < 0 || children[c][0] >= mapShape[0] || children[c][1] < 0 || children[c][1] >= mapShape[1]) { continue; }

                // skip child if it lies on an obstacle
                bool onObstacle = false;
                for (int i = 0; i < obstacleValues.size(); i++) {
                    if (occupancyMap.coeffRef(children[c][0], children[c][1]) == obstacleValues.coeffRef(i)) { onObstacle = true; }
                }

                if (onObstacle) { continue; }

                float g = costs[parent.idx] + euclidean(parentCoord[0], parentCoord[1], children[c][0], children[c][1]);

                int childIdx = index2dTo1d(children[c][0], children[c][1], mapShape[1]);
                if (costs[childIdx] > g) {
                    costs[childIdx] = g;
                    paths[childIdx] = parent.idx;

                    float f = g + epsilon * euclidean(children[c][0], children[c][1], goal.x(), goal.y());
                    openSet.push(Node(f, childIdx));
                }
            }
            numNodesExpanded++;
        }

        int currentIdx = solutionIdx;
        std::stack<int> pathStack;
        while (startIdx != currentIdx) {
            pathStack.push(currentIdx);
            currentIdx = paths[currentIdx];
        }

        // auto path_px = py::ndarray<int>(py::array::ShapeContainer{(ssize_t) pathStack.size(), (ssize_t) 2});
        Eigen::MatrixX2i pathPX = Eigen::MatrixX2i(pathStack.size(), 2);

        int i = 0;
        int coord[2];
        while (!pathStack.empty()) {
            int idx = pathStack.top();
            index1dTo2d(idx, mapShape[1], coord[0], coord[1]);
            pathPX.row(i) << coord[0], coord[1];
            pathStack.pop();
            i++;
        }

        std::cout << "expanded " << numNodesExpanded << " nodes. successful: " << isSuccessful << std::endl;
        return std::make_pair(isSuccessful, pathPX);
    }


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
        std::ostream &ostream) {

        ostream << "start plan, (" << start.coeffRef(0) << ", " << start.coeffRef(1) << ") -> (" << goal.coeffRef(0) << ", " << goal.coeffRef(1) << "), ";

        // todo: we arent going to goal angle.. we need to make sure we collision check it if we want to.
        const float inf = std::numeric_limits<float>::infinity();

        // verify angles are correct (i.e human knows what he is doing when he calls this function)
        std::vector<float> correct_angles = getAstarAngles();
        for (int i = 0; i < 8; i++) {
            if (correct_angles[i] != maskAngles.coeffRef(i)) {
                throw std::logic_error("ERROR: parameter maskAngles of c++ function oriented_astar() does not match required angles. "
                                       "See get_astar_angles() for the correct angles/order. "
                                       "Note, the footprint masks must match these angles, or you will get undesired behavior!");
            }
        }

        // verify planning scale is correct
        if (planningScale < 1) { throw std::logic_error("ERROR: parameter planningScale of c++ function oriented_astar() must be greater than 1 "); }

        // check if goal is reachable
        for (int k = 0; k < obstacleValues.size(); k++) {
            if (occupancyMap.coeffRef(goal.coeffRef(0), goal.coeffRef(1)) == obstacleValues.coeffRef(k)) {
                ostream << "[WARN] goal is obstacle, ";
                break;
            }
        }

        int mapShape[2] = {(int) occupancyMap.rows(), (int) occupancyMap.cols()};
        const int startIdx = index2dTo1d(start.coeffRef(0), start.coeffRef(1), mapShape[1]);

        Node startNode = Node(0.0, startIdx);

        std::priority_queue<Node, std::vector<Node>, std::greater<>> openSet;
        openSet.push(startNode);

        std::vector<float> costs(occupancyMap.size(), inf);
        // std::vector<float> costs((ulong) mapShape[0] * mapShape[1]);
        // for (int i = 0; i < mapShape[0] * mapShape[1]; i++) { costs[i] = inf; }
        costs[startIdx] = 0.0;

        std::vector<int> paths(occupancyMap.size(), -1);
        // std::vector<int> paths((ulong) mapShape[0] * mapShape[1]);
        // for (int i = 0; i < mapShape[0] * mapShape[1]; i++) { paths[i] = -1; }

        std::vector<int> pathsAngleInds(occupancyMap.size(), -1);
        // std::vector<int> paths_angle_inds((ulong) mapShape[0] * mapShape[1]);
        // for (int i = 0; i < mapShape[0] * mapShape[1]; i++) { paths_angle_inds[i] = -1; }

        int children[8][2];
        int parentCoord[2];

        int solutionIdx = -1;
        float closestDistanceToGoal = inf;

        int numNodesExpanded = 0;
        bool isSuccessful = false;
        while (!openSet.empty()) {
            Node parent = openSet.top();
            openSet.pop();

            index1dTo2d(parent.idx, mapShape[1], parentCoord[0], parentCoord[1]);

            float distanceToGoal = euclidean(parentCoord[0], parentCoord[1], goal.x(), goal.y());

            if (distanceToGoal < closestDistanceToGoal) {
                closestDistanceToGoal = distanceToGoal;
                solutionIdx = parent.idx;
            }

            // todo planningScale + delta isnt quite right, maybe max(planningScale, delta) is correct
            if (distanceToGoal <= delta || (planningScale != 1 && distanceToGoal <= std::max(float(planningScale), delta))) {
                isSuccessful = true;
                solutionIdx = parent.idx;
                break;
            }

            // todo Note that if planningScale is too large, in theory this will cause us to jump over thin obstacles.
            // todo In reality, this will never happen, since we won't be planning on a large enough resolutions.
            // todo can I somehow efficiently check the line between the pose and the neighbor to make sure the move is valid?
            children[0][0] = parentCoord[0];
            children[0][1] = parentCoord[1] - planningScale;
            children[1][0] = parentCoord[0] + planningScale;
            children[1][1] = parentCoord[1] - planningScale;
            children[2][0] = parentCoord[0] + planningScale;
            children[2][1] = parentCoord[1];
            children[3][0] = parentCoord[0] + planningScale;
            children[3][1] = parentCoord[1] + planningScale;
            children[4][0] = parentCoord[0];
            children[4][1] = parentCoord[1] + planningScale;
            children[5][0] = parentCoord[0] - planningScale;
            children[5][1] = parentCoord[1] + planningScale;
            children[6][0] = parentCoord[0] - planningScale;
            children[6][1] = parentCoord[1];
            children[7][0] = parentCoord[0] - planningScale;
            children[7][1] = parentCoord[1] - planningScale;

            for (int c = 0; c < 8; c++) {
                // if diagonal is not allowed, skip those children
                if (!allowDiagonal && c % 2 != 0) { continue; }

                // skip child if out of bounds
                if (children[c][0] < 0 || children[c][0] >= mapShape[0] || children[c][1] < 0 || children[c][1] >= mapShape[1]) { continue; }

                // auto child = py::ndarray<int>(py::array::ShapeContainer{2});
                Eigen::Map<Eigen::Vector2i> child = Eigen::Map<Eigen::Vector2i>(children[c]);
                // child.mutable_at(0) = children[c][0];
                // child.mutable_at(1) = children[c][1];

                if (checkForCollision(child, occupancyMap, footprintMasks[c], outlineCoords[c], obstacleValues)) { continue; }

                float g = costs[parent.idx] + euclidean(parentCoord[0], parentCoord[1], children[c][0], children[c][1]);

                int childIdx = index2dTo1d(child.x(), child.y(), mapShape[1]);
                if (costs[childIdx] > g) {
                    costs[childIdx] = g;
                    paths[childIdx] = parent.idx;
                    pathsAngleInds[childIdx] = c;

                    float f = g + epsilon * euclidean(children[c][0], children[c][1], goal.x(), goal.y());
                    openSet.push(Node(f, childIdx));
                }
            }
            numNodesExpanded++;
        }

        int currentIdx = solutionIdx;
        float currentAngle = maskAngles.coeffRef(pathsAngleInds[currentIdx]);
        std::stack<std::pair<int, float>> pathStack;
        while (startIdx != currentIdx) {
            pathStack.push(std::make_pair(currentIdx, currentAngle));
            currentIdx = paths[currentIdx];
            currentAngle = maskAngles.coeffRef(pathsAngleInds[currentIdx]);
        }

        // auto path_px = py::ndarray<float>(py::array::ShapeContainer{(ssize_t) pathStack.size(), (ssize_t) 3});
        auto pathPX = Eigen::MatrixX3f(pathStack.size(), 3);
        int i = 0;
        int coord[2];
        while (!pathStack.empty()) {
            std::pair<int, float> item = pathStack.top();
            index1dTo2d(item.first, mapShape[1], coord[0], coord[1]);
            pathPX.row(i) << float(coord[0]), float(coord[1]), item.second;
            pathStack.pop();
            i++;
        }

        auto n = pathPX.rows();
        ostream << "expanded " << numNodesExpanded << " nodes, path (" << pathPX.coeffRef(0, 0) << ", " << pathPX.coeffRef(0, 1) << ") -> ("
                << pathPX.coeffRef(n - 1, 0) << ", " << pathPX.coeffRef(n - 1, 1) << "), length: " << n << ", successful: " << isSuccessful << std::endl;
        return std::make_pair(isSuccessful, pathPX);
    }

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
        bool parallel) {

        // int cnt = 0;
        // py::assert_ndarray_dims<1>(start);
        // py::assert_ndarray_dims<2>(goals);
        // py::assert_ndarray_dims<2>(occupancy_map);
        // for (auto &footprint_mask: footprint_masks) { py::assert_ndarray_dims<2>(footprint_mask); }
        // py::assert_ndarray_dims<1>(mask_angles);
        // for (auto &outline_coord: outline_coords) { py::assert_ndarray_dims<2>(outline_coord); }
        // py::assert_ndarray_dims<1>(obstacle_values);

        std::vector<std::pair<bool, Eigen::MatrixX3f>> out(goals.rows());
        std::vector<std::pair<bool, std::string>> threadErrorMessages(goals.rows());
        std::vector<std::stringstream> threadLogStreams(goals.rows());

        std::cout << "shape of goals: (" << goals.rows() << ", " << goals.cols() << ")" << std::endl;

#pragma omp parallel for if (parallel) default(none) shared(                                                                                                   \
    out,                                                                                                                                                       \
    threadErrorMessages,                                                                                                                                       \
    threadLogStreams,                                                                                                                                          \
    start,                                                                                                                                                     \
    goals,                                                                                                                                                     \
    occupancyMap,                                                                                                                                              \
    footprintMasks,                                                                                                                                            \
    maskAngles,                                                                                                                                                \
    outlineCoords,                                                                                                                                             \
    obstacleValues,                                                                                                                                            \
    delta,                                                                                                                                                     \
    epsilon,                                                                                                                                                   \
    planningScale,                                                                                                                                             \
    allowDiagonal)
        for (int i = 0; i < goals.rows(); ++i) {
            auto &goal = goals.row(i).transpose();
            threadErrorMessages[i].first = false;
            // auto goal = py::ndarray<int>(py::array::ShapeContainer{2});
            // goal.mutable_at(0) = goals.at(i, 0);
            // goal.mutable_at(1) = goals.at(i, 1);
            try {
                out[i] = orientedAstar(
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
                    allowDiagonal,
                    threadLogStreams[i]);
                // out[i].first = result.first;
                // out[i].second.resize(py::array::ShapeContainer(result.second.shape(), result.second.shape() + result.second.ndim()));
            } catch (const std::logic_error &e) {
                threadErrorMessages[i].first = true;
                threadErrorMessages[i].second = e.what();
            }
        }

        for (auto &threadErrorMessage: threadErrorMessages) {
            if (threadErrorMessage.first) { throw std::logic_error(threadErrorMessage.second); }
        }

        for (auto &threadLogStream: threadLogStreams) { std::cout << threadLogStream.str(); }

        return out;
    }
}  // namespace exploration
