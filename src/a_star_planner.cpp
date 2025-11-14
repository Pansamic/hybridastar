#include <queue>
#include <hybridastar/a_star_planner.h>

const std::array<std::array<int8_t, 2>, 8> AStarPlanner::motion_commands_ =
    {{{0, -1},
      {0, 1},
      {1, 0},
      {-1, 0},
      {-1, -1},
      {-1, 1},
      {1, -1},
      {1, 1}}};

/**
 * @brief Set map parameters for path planning
 * @param map_resolution Map resolution in meters per cell
 * @param map_x_min Minimum x coordinate of map
 * @param map_y_min Minimum y coordinate of map
 * @param map_x_max Maximum x coordinate of map
 * @param map_y_max Maximum y coordinate of map
 * @param map_obstacles Obstacle map matrix
 * @return True if parameters were set successfully
 */
bool AStarPlanner::setMapParameters(float map_resolution, float map_x_min, float map_y_min, float map_x_max, float map_y_max, Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &&map_obstacles)
{
    map_x_min_ = map_x_min;
    map_y_min_ = map_y_min;
    map_x_max_ = map_x_max;
    map_y_max_ = map_y_max;
    map_resolution_ = map_resolution;
    map_grid_rows_ = static_cast<std::size_t>(std::lround((map_x_max_ - map_x_min_) / map_resolution_));
    map_grid_cols_ = static_cast<std::size_t>(std::lround((map_y_max_ - map_y_min_) / map_resolution_));
    if (map_grid_rows_ != map_obstacles.rows() || map_grid_cols_ != map_obstacles.cols())
    {
        return false;
    }
    map_obstacles_ = std::move(map_obstacles);
    return true;
}

/**
 * @brief Plan path from start to goal
 * @param start_pos Start position [x, y]
 * @param goal_pos Goal position [x, y]
 * @return Vector of waypoints forming the path
 */
std::vector<std::array<float, 2>> AStarPlanner::plan(const std::array<float, 2>& start_pos, const std::array<float, 2>& goal_pos)
{
    auto end_node_ptr = findPath(start_pos, goal_pos);
    if (end_node_ptr == nullptr)
    {
        return std::vector<std::array<float, 2>>{};
    }
    return getPathWaypoints(end_node_ptr);
}

/**
 * @brief Find path between start and goal positions
 * @param start_pos Start position [x, y]
 * @param goal_pos Goal position [x, y]
 * @return Pointer to goal node if path found, nullptr otherwise
 */
AStarPlanner::Node *AStarPlanner::findPath(const std::array<float, 2> &start_pos, const std::array<float, 2> &goal_pos)
{
    // Convert the start and goal position to grid index
    auto [start_row, start_col] = calculateGridIndexFromCoordinate(start_pos[0], start_pos[1]);
    auto [goal_row, goal_col] = calculateGridIndexFromCoordinate(goal_pos[0], goal_pos[1]);

    if (!checkGeometry(start_row, start_col) || !checkGeometry(goal_row, goal_col) || checkCollision(start_row, start_col) || checkCollision(goal_row, goal_col))
    {
        return nullptr;
    }

    // Prepare node pool
    node_pool_.clear();
    node_pool_.resize(map_grid_rows_ * map_grid_cols_);

    // Prepare open set
    std::priority_queue<Node *, std::vector<Node *>, Node> openset;

    // Setup start node
    Node &start_node = node_pool_[getNodeID(start_row, start_col)];
    start_node.row = start_row;
    start_node.col = start_col;

    // Setup goal node
    Node &goal_node = node_pool_[getNodeID(goal_row, goal_col)];
    goal_node.row = goal_row;
    goal_node.col = goal_col;

    // Add start node to open set
    start_node.setOpen();
    openset.push(&start_node);

    while (!openset.empty())
    {
        Node *current_node_ptr = openset.top();
        Node &current_node = *current_node_ptr;
        openset.pop();

        if (current_node.isClosed())
        {
            continue;
        }

        current_node.setClosed();

        if (current_node == goal_node)
        {
            return current_node_ptr;
        }

        for (const auto &motion_command : motion_commands_)
        {
            auto expansion_position = getExpandedPosition(current_node, motion_command);
            if (expansion_position[0] == std::numeric_limits<std::size_t>::max() && expansion_position[1] == std::numeric_limits<std::size_t>::max())
            {
                continue;
            }

            Node &successive_node = node_pool_[getNodeID(expansion_position[0], expansion_position[1])];
            if (successive_node.isClosed())
            {
                continue;
            }

            // Calculate costs for the potential new path
            float new_g_cost = calculateGCost(current_node, motion_command);
            float heuristic_cost = calculateHuristicCost(successive_node, goal_node);

            if (successive_node.isUninitialized() || (successive_node.isOpen() && new_g_cost < successive_node.cost_g))
            {
                successive_node.row = expansion_position[0];
                successive_node.col = expansion_position[1];
                successive_node.parent = &current_node;
                successive_node.cost_g = new_g_cost;
                successive_node.cost_h = heuristic_cost;
                successive_node.setOpen();
                openset.push(&successive_node);
            }
        }
    }

    return nullptr;
}

/**
 * @brief Get path length from start to end node
 * @param end_node_ptr Pointer to end node
 * @return Path length, or -1 if node is null
 */
float AStarPlanner::getPathLength(const Node *end_node_ptr)
{
    if (end_node_ptr == nullptr)
    {
        return -1.0;
    }
    return end_node_ptr->cost_g;
}

/**
 * @brief Extract waypoints from path
 * @param end_node_ptr Pointer to end node
 * @return Vector of waypoints as [x, y] coordinates
 */
std::vector<std::array<float, 2>> AStarPlanner::getPathWaypoints(const Node* end_node_ptr)
{
    std::vector<std::array<float, 2>> path;
    const Node *current = end_node_ptr;

    while (current != nullptr)
    {
        // Convert grid indices back to coordinates
        auto [x, y] = calculateCoordinateFromGridIndex(current->row, current->col);

        // For A* path, theta is not meaningful, so set to 0
        path.emplace_back(std::array<float, 2>{x, y});
        current = current->parent;
    }

    // Reverse the path since we built it backwards
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * @brief Get unique node identifier based on position
 * @param row Grid row index
 * @param col Grid column index
 * @return Unique node ID
 */
inline std::size_t AStarPlanner::getNodeID(std::size_t row, std::size_t col) const
{
    // col-major index
    return col * map_grid_rows_ + row;
}

/**
 * @brief Check if grid position is within map bounds
 * @param row Grid row index
 * @param col Grid column index
 * @return True if position is valid
 */
inline bool AStarPlanner::checkGeometry(std::size_t row, std::size_t col) const
{
    return (row >= 0 && row < map_grid_rows_) && (col >= 0 && col < map_grid_cols_);
}

/**
 * @brief Check if grid cell contains obstacle
 * @param row Grid row index
 * @param col Grid column index
 * @return True if cell contains obstacle
 */
inline bool AStarPlanner::checkCollision(std::size_t row, std::size_t col) const
{
    return (map_obstacles_(row, col) == 1);
}

/**
 * @brief Convert grid indices to world coordinates
 * @param row Grid row index
 * @param col Grid column index
 * @return Tuple of (x, y) world coordinates
 */
inline std::tuple<float, float> AStarPlanner::calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const
{
    float x = map_x_max_ - map_resolution_ / 2.0f - row * map_resolution_;
    float y = map_y_max_ - map_resolution_ / 2.0f - col * map_resolution_;
    return std::make_tuple(x, y);
}

/**
 * @brief Convert world coordinates to grid indices
 * @param x World x coordinate
 * @param y World y coordinate
 * @return Tuple of (row, col) grid indices
 */
inline std::tuple<std::size_t, std::size_t> AStarPlanner::calculateGridIndexFromCoordinate(float x, float y) const
{
    std::size_t row = static_cast<std::size_t>(std::floor((map_x_max_ - x) / map_resolution_));
    std::size_t col = static_cast<std::size_t>(std::floor((map_y_max_ - y) / map_resolution_));
    return std::make_tuple(row, col);
}

/**
 * @brief Get expanded position based on motion command
 * @param current_node Current node
 * @param motion_command Motion command [dx, dy]
 * @return Array with new position [row, col]
 */
std::array<std::size_t, 2> AStarPlanner::getExpandedPosition(const Node &current_node, const std::array<int8_t, 2> &motion_command)
{
    std::array<std::size_t, 2> expansion_position;
    expansion_position[0] = current_node.row + motion_command[0];
    expansion_position[1] = current_node.col + motion_command[1];
    if (!checkGeometry(expansion_position[0], expansion_position[1]))
    {
        return {std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max()};
    }
    if (checkCollision(expansion_position[0], expansion_position[1]))
    {
        return {std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max()};
    }
    return expansion_position;
}

/**
 * @brief Calculate movement cost between nodes
 * @param prev_node Previous node
 * @param motion_command Motion command used to reach new node
 * @return Movement cost
 */
float AStarPlanner::calculateGCost(const Node &prev_node, const std::array<int8_t, 2> &motion_command) const
{
    float cost = prev_node.cost_g;
    cost += std::hypot(motion_command[0], motion_command[1]);
    return cost;
}

/**
 * @brief Calculate heuristic cost to goal using diagonal distance
 * @param current_node Current node
 * @param goal_node Goal node
 * @return Heuristic cost
 */
float AStarPlanner::calculateHuristicCost(const Node &current_node, const Node &goal_node) const
{
    static const float sqrt2 = std::sqrt(2.0f);
    std::size_t dx = current_node.row > goal_node.row ? current_node.row - goal_node.row : goal_node.row - current_node.row;
    std::size_t dy = current_node.col > goal_node.col ? current_node.col - goal_node.col : goal_node.col - current_node.col;
    std::size_t square_side_length = std::min(dx, dy);
    std::size_t residual_length = dx > dy ? dx - dy : dy - dx;
    return static_cast<float>(square_side_length) * sqrt2 + static_cast<float>(residual_length);
}