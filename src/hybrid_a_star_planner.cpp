
#include <expected>
#include <queue>
#include <dubins_curve.h>
#include <hybrid_a_star_planner.h>

void HybridAStarPlanner::setConfigParameters(std::size_t num_direction_division, float step_length)
{
    config_num_direction_division_ = num_direction_division;
    config_step_length_ = step_length;
    config_rotation_degree_step_ = 360.0f / static_cast<float>(config_num_direction_division_);
    config_rotation_degree_step_negative_ = 360.0f - config_rotation_degree_step_;
    config_rotation_radian_step_ = 2 * M_PI / static_cast<float>(config_num_direction_division_);
    config_rotation_radian_step_negative_ = 2 * M_PI - config_rotation_radian_step_;
}

void HybridAStarPlanner::setVehicleParameters(
    float vehicle_max_steering_angle,
    float vehicle_steer_precision,
    float vehicle_wheelbase,
    float vehicle_axle_to_front,
    float vehicle_axle_to_rear,
    float vehicle_width,
    bool vehicle_enable_reverse)
{
    vehicle_max_steering_angle_ = vehicle_max_steering_angle;
    vehicle_steer_precision_ = vehicle_steer_precision;
    vehicle_wheelbase_ = vehicle_wheelbase;
    vehicle_axle_to_front_ = vehicle_axle_to_front;
    vehicle_axle_to_rear_ = vehicle_axle_to_rear;
    vehicle_width_ = vehicle_width;
    vehicle_enable_reverse_ = vehicle_enable_reverse;

    initializeMotionCommands();
}

void HybridAStarPlanner::setCostWeights(
    float cost_weight_reverse,
    float cost_weight_direction_change,
    float cost_weight_steering_angle,
    float cost_weight_steering_rate,
    float cost_weight_hybrid)
{
    cost_weight_reverse_ = cost_weight_reverse;
    cost_weight_direction_change_ = cost_weight_direction_change;
    cost_weight_steering_angle_ = cost_weight_steering_angle;
    cost_weight_steering_rate_ = cost_weight_steering_rate;
    cost_weight_hybrid_ = cost_weight_hybrid;
}

HybridAStarPlanner::ErrorCode HybridAStarPlanner::setMapParameters(
    float map_resolution,
    float map_x_min,
    float map_y_min,
    float map_x_max,
    float map_y_max,
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>&& map_obstacles)
{
    map_resolution_ = map_resolution;
    map_x_min_ = map_x_min;
    map_y_min_ = map_y_min;
    map_x_max_ = map_x_max;
    map_y_max_ = map_y_max;
    map_grid_rows_ = static_cast<std::size_t>(std::ceil((map_y_max - map_y_min) / map_resolution));
    map_grid_cols_ = static_cast<std::size_t>(std::ceil((map_x_max - map_x_min) / map_resolution));

    if (map_grid_rows_ != map_obstacles.rows() || map_grid_cols_ != map_obstacles.cols())
    {
        return ErrorCode::INVALID_ARGUMENT;
    }

    map_obstacles_ = std::move(map_obstacles);

    std::vector<std::vector<float>> obstacles;
    for (std::size_t col = 0; col < map_obstacles_.cols(); col++)
    {
        for (std::size_t row = 0; row < map_obstacles_.rows(); row++)
        {
            if (map_obstacles_(row, col) == 1)
            {
                auto [x, y] = calculateCoordinateFromGridIndex(row, col);

                // constuct vector in-place to prevent memory copying
                obstacles.emplace_back(std::vector<float>{x, y});

                // it's wrong to construct vector with move semantics because temporary object
                // is created on stack which will be destroyed when it goes out of scope
                // std::vector<float> obstacle_vector(x, y);
                // obstacles.emplace_back(std::move(obstacle_vector));
            }
        }
    }

    obstacle_tree_.reset();
    obstacle_tree_ = std::make_unique<KDTree>(obstacles);

    return ErrorCode::SUCCESS;
}

std::expected<const HybridAStarPlanner::Node*, HybridAStarPlanner::ErrorCode>
HybridAStarPlanner::findPath(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state)
{
    // Setup a maximum iteration count to avoid infinite loops
    const std::size_t max_iterations = 10000;
    std::size_t iterations = 0;

    // Pre-allocate large memory block to avoid scattered memory allocations
    node_pool_.clear();
    node_pool_.resize(map_grid_rows_ * map_grid_cols_ * config_num_direction_division_);

    std::priority_queue<Node*, std::vector<Node*>, Node> openset;

    Node& start_node = node_pool_[getNodeID(start_state[0], start_state[1], start_state[2])];
    start_node.x = start_state[0];
    start_node.y = start_state[1];
    start_node.yaw = start_state[2];
    // start node has no motion command

    Node &goal_node = node_pool_[getNodeID(goal_state[0], goal_state[1], goal_state[2])];
    goal_node.x = goal_state[0];
    goal_node.y = goal_state[1];
    goal_node.yaw = goal_state[2];

    start_node.setOpen();
    openset.push(&start_node);

    while(!openset.empty() && iterations < max_iterations)
    {
        iterations++;

        // Get the node with the lowest cost
        Node* current_node_ptr = openset.top();
        Node& current_node = *current_node_ptr;

        if (current_node.isClosed())
        {
            // Remove lowest cost node from the open set
            openset.pop();
            continue;
        }

        // Mark node as closed
        current_node.setClosed();
        // Remove lowest cost node from the open set
        openset.pop();

        if (isNodeXYTEqual(current_node, goal_node))
        {
            return current_node_ptr;
        }

        // Search for dubins shot
        auto dubins_shot_result = getDubinsShot(current_node_ptr, &goal_node);
        if (dubins_shot_result.has_value())
        {
            const Node* dubins_goal_node_ptr = dubins_shot_result.value();
            if (isNodeXYTEqual(*dubins_goal_node_ptr, goal_node))
            {
                return dubins_goal_node_ptr;
            }
        }

        // Search for forward simulation
        for (auto& motion_command : motion_commands_)
        {
            auto expansion_result = getExpandedState(*current_node_ptr, motion_command);
            if (!expansion_result.has_value())
            {
                // Error Handling
                continue;
            }
            std::array<float, 3> expansion_state = expansion_result.value();

            // If node is out of map or collide with obstacles
            if (!checkGeometry(expansion_state[0], expansion_state[1], expansion_state[2]) || checkCollision(expansion_state[0], expansion_state[1], expansion_state[2]))
            {
                // Process invalid node
                continue;
            }

            Node& successive_node = node_pool_[getNodeID(expansion_state[0], expansion_state[1], expansion_state[2])];

            if (successive_node.isClosed())
            {
                continue;
            }

            // Build complete node.
            successive_node.x = expansion_state[0];
            successive_node.y = expansion_state[1];
            successive_node.yaw = expansion_state[2];
            successive_node.cost_g = calculateGCost(current_node, motion_command);
            successive_node.cost_h = calculateHuristicCost(successive_node, goal_node);
            successive_node.motion = &motion_command;
            successive_node.parent = current_node_ptr;

            if (successive_node.isUninitialized() || (successive_node.isOpen() && (successive_node.cost_h + successive_node.cost_g) < (current_node.cost_h + current_node.cost_g)))
            {
                successive_node.setOpen();
                openset.push(&successive_node);
            }
        }
    }
    return nullptr;
}

std::size_t HybridAStarPlanner::getNodeID(float x, float y, float yaw)
{
    auto [row, col] = calculateGridIndexFromCoordinate(x, y);

    return (row * config_num_direction_division_ + col * map_grid_rows_ * config_num_direction_division_ + static_cast<std::size_t>(getPositiveNormalizedRadianAngle(yaw) / config_rotation_radian_step_));
}

void HybridAStarPlanner::initializeMotionCommands()
{
    motion_commands_.clear();

    for (float angle = -vehicle_max_steering_angle_; angle <= vehicle_max_steering_angle_; angle += vehicle_max_steering_angle_ / vehicle_steer_precision_)
    {
        motion_commands_.emplace_back(angle, 1);
        motion_commands_.emplace_back(angle, -1);
    }
}

std::tuple<float, float> HybridAStarPlanner::calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const
{
    float x = map_resolution_ / 2.0f + map_x_min_ + row * map_resolution_;
    float y = map_resolution_ / 2.0f + map_y_min_ + col * map_resolution_;
    return std::make_tuple(x, y);
}

std::tuple<std::size_t, std::size_t> HybridAStarPlanner::calculateGridIndexFromCoordinate(float x, float y) const
{
    std::size_t row = static_cast<std::size_t>(std::floor((x - map_x_min_) / map_resolution_));
    std::size_t col = static_cast<std::size_t>(std::floor((y - map_y_min_) / map_resolution_));
    return std::make_tuple(row, col);
}

float HybridAStarPlanner::calculateGCost(const Node& prev_node, const MotionCommand& motion_command) const
{
    // Inherit G cost from parent
    float cost = prev_node.cost_g;
    
    // Cost1: step length
    cost += config_step_length_ * (motion_command.direction == -1) ? cost_weight_reverse_: 1;

    // Cost2: direction variance
    // check motion existance to prevent null pointer because start node has no motion command
    if (prev_node.motion != nullptr)
    {
        cost += (prev_node.motion->direction != motion_command.direction) ? cost_weight_direction_change_ : 0;
    }

    // Cost3: steering angle
    cost += std::abs(motion_command.rotation) * cost_weight_steering_angle_;

    // Cost4: steering angle variance
    // check motion existance to prevent null pointer because start node has no motion command
    if (prev_node.motion != nullptr)
    {
        cost += std::abs(motion_command.rotation - prev_node.motion->rotation) * cost_weight_steering_rate_;
    }

    return cost;
}

float HybridAStarPlanner::calculateHuristicCost(const Node& current_node, const Node& goal_node) const
{
    // Calculate 2D A star distance.
    std::array<float, 2> start_pos_2d{current_node.x, current_node.y};
    std::array<float, 2> goal_pos_2d{goal_node.x, goal_node.y};
    
    // Create a temporary AStarPlanner instance with the map parameters
    AStarPlanner a_star_planner;
    auto map_obstacles_copy = map_obstacles_;
    a_star_planner.setMapParameters(map_resolution_, map_x_min_, map_y_min_, map_x_max_, map_y_max_, std::move(map_obstacles_copy));
    auto a_star_result = a_star_planner.findPath(start_pos_2d, goal_pos_2d);
    if (!a_star_result.has_value())
    {
        return std::numeric_limits<float>::max();
    }
    auto a_star_end_node_ptr = a_star_result.value();
    auto a_star_path_result = a_star_planner.getPathLength(a_star_end_node_ptr);
    if (!a_star_path_result.has_value())
    {
        return std::numeric_limits<float>::max();
    }
    float distance_cost = a_star_path_result.value();
    // To avoid creating a new A* planner every time, we'll use a more efficient huristic like Euclidean distance
    // float dx = goal_node.x - current_node.x;
    // float dy = goal_node.y - current_node.y;
    // float distance_cost = std::sqrt(dx * dx + dy * dy);
    
    // Calculate dubins curve cost.
    double vehicle_turning_radius = vehicle_wheelbase_ / std::tan(vehicle_max_steering_angle_);
    DubinsCurve dubins;
    std::array<double, 3> dubins_start_state{current_node.x, current_node.y, current_node.yaw};
    std::array<double, 3> dubins_goal_state{goal_node.x, goal_node.y, goal_node.yaw};
    dubins.init(dubins_start_state, dubins_goal_state, vehicle_turning_radius);
    double dubins_cost = dubins.getPathLength();

    return std::max(distance_cost, static_cast<float>(dubins_cost));
}

std::expected<std::array<float, 3>, HybridAStarPlanner::ErrorCode>
HybridAStarPlanner::getExpandedState(const Node& current_node, const MotionCommand& motion_command)
{
    std::array<float, 3> expanded_state;

    const float& current_x = current_node.x;
    const float& current_y = current_node.y;
    const float& current_yaw = current_node.yaw;
    const float move_dir = static_cast<float>(motion_command.direction);
    const float& move_rot = motion_command.rotation;

    expanded_state[0] = current_x + config_step_length_ * std::cos(current_yaw) * move_dir;
    expanded_state[1] = current_y + config_step_length_ * std::sin(current_yaw) * move_dir;
    expanded_state[2] = getPositiveNormalizedRadianAngle(current_yaw + move_rot);
    
    return expanded_state;
}

std::expected<HybridAStarPlanner::Node*, HybridAStarPlanner::ErrorCode>
HybridAStarPlanner::getDubinsShot(Node* start_node, Node* goal_node)
{
    DubinsCurve dubins;

    double vehicle_turning_radius = vehicle_wheelbase_ / std::tan(vehicle_max_steering_angle_);
    
    std::array<double, 3> start_state{static_cast<double>(start_node->x),
                                      static_cast<double>(start_node->y),
                                      static_cast<double>(start_node->yaw)};
    std::array<double, 3> goal_state {static_cast<double>(goal_node->x),
                                      static_cast<double>(goal_node->y),
                                      static_cast<double>(goal_node->yaw)};
    if (dubins.init(start_state, goal_state, vehicle_turning_radius))
    {
        return std::unexpected(ErrorCode::NO_PATH);
    }

    const float& dubins_step_length = map_resolution_;

    float dubins_sample_length = dubins_step_length;

    float dubins_path_length = dubins.getPathLength();

    dubins_path_node_pool_.clear();
    dubins_path_node_pool_.reserve(static_cast<std::size_t>(dubins_path_length / dubins_step_length));

    bool is_first = true;
    while(dubins_sample_length < dubins_path_length)
    {
        std::array<double, 3> state;
        dubins.pathSample(dubins_sample_length, state);
        Node& node = dubins_path_node_pool_.emplace_back(state[0], state[1], state[2]);
        if (is_first)
        {
            node.parent = start_node;
            is_first = false;
        }
        else
        {
            node.parent = &dubins_path_node_pool_[dubins_path_node_pool_.size()-2];
        }
        dubins_sample_length += dubins_step_length;
    }
    return &dubins_path_node_pool_[dubins_path_node_pool_.size()-1];
}

bool HybridAStarPlanner::checkGeometry(float x, float y, float yaw) const
{
    // Calculate the half dimensions of the vehicle
    float half_width = vehicle_width_ / 2.0f;
    float half_length = (vehicle_axle_to_front_ + vehicle_axle_to_rear_) / 2.0f;
    
    // Calculate the positions of the four corners of the vehicle
    // Using rotation matrix to transform from vehicle frame to world frame
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    
    // Four corners relative to vehicle center
    // Front-left corner
    float fl_x = x + (half_length * cos_yaw - half_width * sin_yaw);
    float fl_y = y + (half_length * sin_yaw + half_width * cos_yaw);
    
    // Front-right corner
    float fr_x = x + (half_length * cos_yaw + half_width * sin_yaw);
    float fr_y = y + (half_length * sin_yaw - half_width * cos_yaw);
    
    // Rear-left corner
    float rl_x = x + (-half_length * cos_yaw - half_width * sin_yaw);
    float rl_y = y + (-half_length * sin_yaw + half_width * cos_yaw);
    
    // Rear-right corner
    float rr_x = x + (-half_length * cos_yaw + half_width * sin_yaw);
    float rr_y = y + (-half_length * sin_yaw - half_width * cos_yaw);
    
    // Check if all corners are within map boundaries
    bool fl_in_bounds = (fl_x >= map_x_min_ && fl_x <= map_x_max_ && fl_y >= map_y_min_ && fl_y <= map_y_max_);
    bool fr_in_bounds = (fr_x >= map_x_min_ && fr_x <= map_x_max_ && fr_y >= map_y_min_ && fr_y <= map_y_max_);
    bool rl_in_bounds = (rl_x >= map_x_min_ && rl_x <= map_x_max_ && rl_y >= map_y_min_ && rl_y <= map_y_max_);
    bool rr_in_bounds = (rr_x >= map_x_min_ && rr_x <= map_x_max_ && rr_y >= map_y_min_ && rr_y <= map_y_max_);
    
    return fl_in_bounds && fr_in_bounds && rl_in_bounds && rr_in_bounds;
}

bool HybridAStarPlanner::checkCollision(float x, float y, float yaw) const
{
    float dl = (vehicle_axle_to_front_ - vehicle_axle_to_rear_) / 2.0;

    float vehicle_length = vehicle_axle_to_front_ + vehicle_axle_to_rear_;

    float vehicle_circle_radius = std::hypot(vehicle_length, vehicle_width_) / 2.0f;

    std::vector<float> circle_center{x + dl * std::cos(yaw), y + dl * std::sin(yaw)};
    
    auto neighborhood = obstacle_tree_->neighborhood_points(circle_center, vehicle_circle_radius);

    // If no obstacles in the circumcircle, no collision
    if (neighborhood.empty())
    {
        return false;
    }

    for (const auto &point : neighborhood)
    {
        // Compute vector from the center of circumcircle of vehicle to obstacle point in map frame.
        float vec_world_x = point[0] - circle_center[0];
        float vec_world_y = point[1] - circle_center[1];

        // Apply rotation transform
        // Compute vector from the center of circumcircle of vehicle to obstacle point in vehicle(body) frame.
        float angle = -yaw;
        float vec_body_x = std::cos(angle) * vec_world_x + std::sin(angle) * vec_world_y;
        float vec_body_y = -std::sin(angle) * vec_world_x + std::cos(angle) * vec_world_y;

        if (vec_body_x <=  vehicle_length / 2.0 &&
            vec_body_x >= -vehicle_length / 2.0 &&
            vec_body_y <=  vehicle_width_ / 2.0 &&
            vec_body_y >= -vehicle_width_ / 2.0)
        {
            return true;
        }
    }
    return false; // No collision
}

inline float HybridAStarPlanner::getPositiveNormalizedRadianAngle(const float& angle)
{
    if(angle >= 0.0 && angle <= 2.0 * M_PI)
    {
        return angle;
    }
    if(angle >= -2.0 * M_PI && angle < 0.0)
    {
        return angle + 2.0 * M_PI;
    }
    float result = std::fmod(angle, 2.0 * M_PI);
    if (result < 0.0)
    {
        result += 2.0 * M_PI;
    }
    return result;
}

inline bool HybridAStarPlanner::isNodeXYTEqual(const Node& node_a, const Node& node_b)
{
    return (static_cast<int>(node_a.x / map_resolution_) == static_cast<int>(node_b.x / map_resolution_)) &&
            (static_cast<int>(node_a.y / map_resolution_) == static_cast<int>(node_b.y / map_resolution_)) &&
            (std::abs(node_a.yaw - node_b.yaw) <= config_rotation_radian_step_ ||
            std::abs(node_a.yaw - node_b.yaw) >= config_rotation_radian_step_negative_);
}

std::vector<std::array<float, 3>> HybridAStarPlanner::getPathWaypoints(const Node* end_node_ptr)
{
    if (end_node_ptr == nullptr)
    {
        return std::vector<std::array<float, 3>>();
    }
    std::vector<std::array<float, 3>> path;
    const Node* current_node_ptr = end_node_ptr;
    while(current_node_ptr != nullptr)
    {
        path.push_back(std::array<float, 3>{current_node_ptr->x, current_node_ptr->y, current_node_ptr->yaw});
        current_node_ptr = current_node_ptr->parent;
    }
    return path;
}

std::vector<std::array<float, 3>> HybridAStarPlanner::plan(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state)
{
    auto find_path_result = findPath(start_state, goal_state);
    if (!find_path_result.has_value())
    {
        return std::vector<std::array<float, 3>>();
    }
    auto end_node_ptr = find_path_result.value();
    return getPathWaypoints(end_node_ptr);
}