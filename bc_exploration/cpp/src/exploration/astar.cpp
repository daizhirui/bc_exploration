#include "exploration/astar.hpp"
#include "exploration/util.hpp"
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>

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
        std::vector<float> correctAngles = getAstarAngles();
        for (int i = 0; i < 8; i++) {
            if (correctAngles[i] != maskAngles.coeffRef(i)) {
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
        costs[startIdx] = 0.0;

        std::vector<int> paths(occupancyMap.size(), -1);
        std::vector<int> pathsAngleInds(occupancyMap.size(), -1);

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

                Eigen::Map<Eigen::Vector2i> child = Eigen::Map<Eigen::Vector2i>(children[c]);

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

        auto pathPX = Eigen::MatrixX3f(pathStack.size(), 3);  // FIXME: may be ZERO!!!
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
        ostream << "expanded " << numNodesExpanded << " nodes";
        if (n > 0) {
            ostream << ", path (" << pathPX.coeffRef(0, 0) << ", " << pathPX.coeffRef(0, 1) << ") -> (" << pathPX.coeffRef(n - 1, 0) << ", "
                    << pathPX.coeffRef(n - 1, 1) << "), length: " << n;
        }
        ostream << ", successful: " << isSuccessful << std::endl;
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
            } catch (const std::logic_error &e) {
                threadErrorMessages[i].first = true;
                threadErrorMessages[i].second = e.what();
            }
        }

        for (auto &threadErrorMessage: threadErrorMessages) {
            if (threadErrorMessage.first) { throw std::logic_error(threadErrorMessage.second); }
        }

        for (int i = 0; i < goals.rows(); ++i) { std::cout << i << ":, " << threadLogStreams[i].str(); }

        return out;
    }

    static float
    computePathLength(const Eigen::MatrixX3f &path) {
        auto m = path.rows();
        return (path.block(0, 0, m - 1, 2) - path.block(1, 0, m - 1, 2)).rowwise().norm().sum();
    }

    struct SharedResource {
        // read-only resource
        const Eigen::Ref<const Eigen::Vector2i> &start;
        const Eigen::Ref<const Eigen::MatrixX2i> &goals;
        const Eigen::Ref<const Eigen::VectorXf> &goalInitPriorities;
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap;
        const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks;
        const Eigen::Ref<const Eigen::VectorXf> &maskAngles;
        const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords;
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues;
        float delta;
        float epsilon;
        int planningScale;
        bool allowDiagonal;
        // read-write resource
        std::vector<bool> skipFlags;
        std::vector<float> newHeuristics;
        std::vector<std::pair<bool, Eigen::MatrixX3f>> out;
        std::vector<std::pair<bool, std::string>> threadErrorMessages;
        std::vector<std::stringstream> threadLogStreams;

        void
        updateNewHeuristic(int i) {
            if (out[i].first) { newHeuristics[i] = goalInitPriorities[i] / computePathLength(out[i].second); }
        }
    };

    static std::mutex resourceMutex;
    // protected by lock
    struct ProtectedResource {
        float maxFeasibleHeuristic = 0;
        std::vector<int> computeOrders;
        std::vector<float> heuristics;
        std::function<bool(const int &, const int &)> cmp = [&](const int &left, const int &right) { return heuristics[left] < heuristics[right]; };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> priorityQueue;
        std::vector<int> inProcessIndices;
        int numInProcessIndices = 0;
        bool exit = false;

        ProtectedResource()
            : priorityQueue(cmp) {}

        void
        reset(size_t n, size_t numThreads) {
            maxFeasibleHeuristic = 0;
            computeOrders.clear();
            heuristics.resize(n);
            priorityQueue = decltype(priorityQueue)(cmp);
            inProcessIndices.resize(numThreads, -1);
            exit = false;
        }

        float
        maxHeuristicInProcess() {
            float out = 0;
            numInProcessIndices = 0;
            for (auto &index: inProcessIndices) {
                if (index != -1) {
                    numInProcessIndices++;
                    if (heuristics[index] > out) { out = heuristics[index]; }
                }
            }
            return out;
        }
    };

    ProtectedResource protectedResource;

    static void
    threadWorkerForOrientedAstarPrioritizedMultiGoals(int threadId, const std::shared_ptr<SharedResource> &sharedResource) {

        bool running;
        int index;
        bool computeIt;
        do {
            {
                std::unique_lock<std::mutex> lk(resourceMutex);
                running = !protectedResource.priorityQueue.empty() && !protectedResource.exit;
                if (running) {
                    index = protectedResource.priorityQueue.top();
                    protectedResource.inProcessIndices[threadId] = index;
                    protectedResource.priorityQueue.pop();
                    protectedResource.computeOrders.push_back(index);
                    computeIt = protectedResource.maxFeasibleHeuristic < protectedResource.heuristics[index];
                }
            }

            if (running) {
                if (computeIt) {

                    try {
                        auto &start = sharedResource->start;
                        auto &goal = sharedResource->goals.row(index).transpose();
                        auto &occupancyMap = sharedResource->occupancyMap;
                        auto &footprintMasks = sharedResource->footprintMasks;
                        auto &maskAngles = sharedResource->maskAngles;
                        auto &outlineCoords = sharedResource->outlineCoords;
                        auto &obstacleValues = sharedResource->obstacleValues;
                        auto &delta = sharedResource->delta;
                        auto &epsilon = sharedResource->epsilon;
                        auto &planningScale = sharedResource->planningScale;
                        auto &allowDiagonal = sharedResource->allowDiagonal;
                        auto &ostream = sharedResource->threadLogStreams[index];

                        ostream << "start plan, (" << start.coeffRef(0) << ", " << start.coeffRef(1) << ") -> (" << goal.coeffRef(0) << ", " << goal.coeffRef(1)
                                << "), ";

                        // todo: we arent going to goal angle.. we need to make sure we collision check it if we want to.
                        const float inf = std::numeric_limits<float>::infinity();

                        // verify angles are correct (i.e human knows what he is doing when he calls this function)
                        std::vector<float> correctAngles = getAstarAngles();
                        for (int i = 0; i < 8; i++) {
                            if (correctAngles[i] != maskAngles.coeffRef(i)) {
                                throw std::logic_error("ERROR: parameter maskAngles of c++ function oriented_astar() does not match required angles. "
                                                       "See get_astar_angles() for the correct angles/order. "
                                                       "Note, the footprint masks must match these angles, or you will get undesired behavior!");
                            }
                        }

                        // verify planning scale is correct
                        if (planningScale < 1) {
                            throw std::logic_error("ERROR: parameter planningScale of c++ function oriented_astar() must be greater than 1 ");
                        }

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
                        costs[startIdx] = 0.0;

                        std::vector<int> paths(occupancyMap.size(), -1);
                        std::vector<int> pathsAngleInds(occupancyMap.size(), -1);

                        int children[8][2];
                        int parentCoord[2];

                        int solutionIdx = -1;
                        float closestDistanceToGoal = inf;

                        int numNodesExpanded = 0;
                        bool isSuccessful = false;
                        while (!openSet.empty()) {
                            if (numNodesExpanded % 100 == 0) {
                                {
                                    std::unique_lock<std::mutex> lk(resourceMutex);
                                    if (protectedResource.exit) { return; }
                                }
                            }

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

                                Eigen::Map<Eigen::Vector2i> child = Eigen::Map<Eigen::Vector2i>(children[c]);

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
                        ostream << "expanded " << numNodesExpanded << " nodes";
                        if (n > 0) {
                            ostream << ", path (" << pathPX.coeffRef(0, 0) << ", " << pathPX.coeffRef(0, 1) << ") -> (" << pathPX.coeffRef(n - 1, 0) << ", "
                                    << pathPX.coeffRef(n - 1, 1) << "), length: " << n;
                        }
                        ostream << ", successful: " << isSuccessful;

                        sharedResource->out[index].first = isSuccessful;
                        sharedResource->out[index].second = pathPX;
                        sharedResource->updateNewHeuristic(index);

                        {
                            std::unique_lock<std::mutex> lk(resourceMutex);
                            if (sharedResource->newHeuristics[index] > protectedResource.heuristics[index]) {
                                std::cout << index << ": update heuristic prediction from " << protectedResource.heuristics[index] << " to "
                                          << sharedResource->newHeuristics[index] << std::endl;
                                protectedResource.heuristics[index] = sharedResource->newHeuristics[index];
                            }
                            if (sharedResource->newHeuristics[index] > protectedResource.maxFeasibleHeuristic) {
                                protectedResource.maxFeasibleHeuristic = sharedResource->newHeuristics[index];
                            }
                            protectedResource.inProcessIndices[threadId] = -1;
                            std::cout << index << ": heuristic " << protectedResource.heuristics[index] << " -> " << sharedResource->newHeuristics[index]
                                      << ", maxFeasibleHeuristic = " << protectedResource.maxFeasibleHeuristic << std::endl;
                        }

                        sharedResource->skipFlags[index] = false;
                    } catch (const std::logic_error &e) {
                        sharedResource->threadErrorMessages[index].first = true;
                        sharedResource->threadErrorMessages[index].second = e.what();
                    }
                } else {
                    sharedResource->out[index].first = false;
                    sharedResource->out[index].second = Eigen::MatrixX3f(0, 3);
                }
            }
        } while (running);
    }

    std::vector<std::pair<bool, Eigen::MatrixX3f>>
    orientedAstarPrioritizedMultiGoals(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::MatrixX2i> &goals,
        const Eigen::Ref<const Eigen::VectorXf> &goalInitPriorities,  // heuristic = goalInitPriority / distance_to_goal
        const Eigen::Ref<const Eigen::MatrixX<uint8_t>> &occupancyMap,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<bool>>> &footprintMasks,
        const Eigen::Ref<const Eigen::VectorXf> &maskAngles,
        const std::vector<Eigen::Ref<const Eigen::MatrixX<int>>> &outlineCoords,
        const Eigen::Ref<const Eigen::VectorX<uint8_t>> &obstacleValues,
        float delta,
        float epsilon,
        int planningScale,
        bool allowDiagonal) {

        auto n = goals.rows();
#if defined(NDEBUG)
        auto numThreads = std::thread::hardware_concurrency();
#else
        ssize_t numThreads = 2;
#endif
        if (n < numThreads) { numThreads = n; }

        std::cout << "shape of goals: (" << n << ", " << goals.cols() << ")" << std::endl;

        protectedResource.reset(n, numThreads);
        for (int i = 0; i < n; ++i) {
            protectedResource.heuristics[i] = goalInitPriorities.coeffRef(i) / (start - goals.row(i).transpose()).cast<float>().norm();
            protectedResource.priorityQueue.push(i);
        }

        std::cout << "maxPredictedHeuristic: " << protectedResource.heuristics[protectedResource.priorityQueue.top()] << std::endl;

        auto sharedResource = std::make_shared<SharedResource>(SharedResource{
            start,
            goals,
            goalInitPriorities,
            occupancyMap,
            footprintMasks,
            maskAngles,
            outlineCoords,
            obstacleValues,
            delta,
            epsilon,
            planningScale,
            allowDiagonal,
            std::vector<bool>(n, true),                         // skipFlags
            std::vector<float>(n, 0),                           // newHeuristics
            std::vector<std::pair<bool, Eigen::MatrixX3f>>(n),  // out
            std::vector<std::pair<bool, std::string>>(n),       // threadErrorMessages
            std::vector<std::stringstream>(n)});                // threadLogStreams

        std::vector<std::thread> threads;
        threads.reserve(numThreads);
        for (size_t i = 0; i < numThreads; ++i) { threads.emplace_back(&threadWorkerForOrientedAstarPrioritizedMultiGoals, i, sharedResource); }

        bool wait = true;
        while (wait) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            {
                std::unique_lock<std::mutex> lk(resourceMutex);
                auto maxHeuristicInProcess = protectedResource.maxHeuristicInProcess();
                if (protectedResource.maxFeasibleHeuristic >= maxHeuristicInProcess * 1.1) {
                    wait = false;
                    protectedResource.exit = true;
                    if (protectedResource.numInProcessIndices > 0) {
                        std::cout << "exit in advance, maxFeasibleHeuristic = " << protectedResource.maxFeasibleHeuristic << std::endl;
                    }
                }
            }
        }

        for (auto &thread: threads) { thread.join(); }

        for (auto &threadErrorMessage: sharedResource->threadErrorMessages) {
            if (threadErrorMessage.first) { throw std::logic_error(threadErrorMessage.second); }
        }

        for (int i = 0; i < n; ++i) {
            std::cout << i << ": initPriority=" << goalInitPriorities[i] << ", heuristic=" << protectedResource.heuristics[i]
                      << ", newHeuristic=" << sharedResource->newHeuristics[i] << ", " << sharedResource->threadLogStreams[i].str();

            if (sharedResource->skipFlags[i]) {
                std::cout << " [skipped]." << std::endl;
            } else {
                std::cout << " [done]." << std::endl;
            }
        }

        std::cout << "compute order: ";
        for (auto &i: protectedResource.computeOrders) { std::cout << i << ", "; }
        std::cout << std::endl;

        return sharedResource->out;
    }

}  // namespace exploration
