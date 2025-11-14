#ifndef __HYBRID_A_STAR_PLANNER_H__
#define __HYBRID_A_STAR_PLANNER_H__ 

#include <cstdint>
#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <hybridastar/kdtree.hpp>
#include <hybridastar/a_star_planner.h>

class HybridAStarPlanner final
{
public:
    /**
     * @brief Motion command representation for vehicle movement
     */
    struct MotionCommand
    {
        float rotation;    ///< Rotation angle in radians
        int direction;     ///< Direction (1: forward, -1: backward)

        /**
         * @brief Default constructor
         */
        MotionCommand()
        {
            this->rotation = 0.0;
            this->direction = 1;
        }

        /**
         * @brief Constructor with parameters
         * @param rotation Rotation angle in radians
         * @param direction Direction (1: forward, -1: backward)
         */
        MotionCommand(float rotation, int direction)
        {
            this->rotation = rotation;
            this->direction = direction;
        }

        /**
         * @brief Assignment operator
         * @param motion_command Motion command to copy from
         */
        void operator=(const MotionCommand& motion_command)
        {
            this->rotation = motion_command.rotation;
            this->direction = motion_command.direction;
        }
    };

    /**
     * @brief Node representation for Hybrid A* algorithm
     */
    struct Node
    {
        float x;                    ///< X coordinate
        float y;                    ///< Y coordinate
        float yaw;                  ///< Yaw angle in radians
        float cost_g;               ///< Actual cost from start node
        float cost_h;               ///< Heuristic cost to goal node
        uint8_t status;             ///< Node status (0: uninitialized, 1: open, 2: closed)
        const MotionCommand* motion;///< Motion command used to reach this node
        const Node* parent;         ///< Pointer to parent node

        /**
         * @brief Default constructor
         */
        Node()
        {
            this->x = 0.0;
            this->y = 0.0;
            this->yaw = 0.0;
            this->cost_g = 0.0;
            this->cost_h = 0.0;
            this->status = 0; // set uninitialized
            this->motion = nullptr;
            this->parent = nullptr;
        }

        /**
         * @brief Constructor with position and orientation
         * @param x X coordinate
         * @param y Y coordinate
         * @param yaw Yaw angle in radians
         */
        Node(float x, float y, float yaw)
        {
            this->x = x;
            this->y = y;
            this->yaw = yaw;
            this->cost_g = 0.0f;
            this->cost_h = 0.0f;
            this->status = 0; // set uninitialized
            this->motion = nullptr;
            this->parent = nullptr;
        }

        /**
         * @brief Copy constructor
         * @param node Node to copy from
         */
        Node(const Node& node)
        {
            this->x = node.x;
            this->y = node.y;
            this->yaw = node.yaw;
            this->cost_g = node.cost_g;
            this->cost_h = node.cost_h;
            this->status = node.status;
            this->motion = node.motion;
            this->parent = node.parent;
        }

        /**
         * @brief Check if node is uninitialized
         * @return True if node is uninitialized
         */
        bool isUninitialized() const {return this->status == 0;}

        /**
         * @brief Check if node is in open set
         * @return True if node is in open set
         */
        bool isOpen() const {return this->status == 1;}

        /**
         * @brief Check if node is in closed set
         * @return True if node is in closed set
         */
        bool isClosed() const {return this->status == 2;}

        /**
         * @brief Set node status to open
         */
        void setOpen() {this->status = 1;}

        /**
         * @brief Set node status to closed
         */
        void setClosed() {this->status = 2;}

        /**
         * @brief Comparison operator for priority queue ordering
         * @param lhs Left-hand side node pointer
         * @param rhs Right-hand side node pointer
         * @return True if lhs has higher cost than rhs
         */
        bool operator()(const Node* lhs, const Node* rhs) const
        {
            return (lhs->cost_g + lhs->cost_h) > (rhs->cost_g + rhs->cost_h);
        }
    };

    /**
     * @brief Default constructor
     */
    explicit HybridAStarPlanner() = default;

    /**
     * @brief Destructor
     */
    ~HybridAStarPlanner() = default;
    
    /**
     * @brief Set configuration parameters
     * @param num_direction_division Number of direction divisions
     * @param step_length Step length for each movement
     */
    void setConfigParameters(std::size_t num_direction_division, float step_length);

    /**
     * @brief Set vehicle parameters
     * @param vehicle_max_steering_angle Maximum steering angle in radians
     * @param vehicle_steer_precision Steering precision
     * @param vehicle_wheelbase Wheelbase of the vehicle
     * @param vehicle_axle_to_front Distance from rear axle to front of vehicle
     * @param vehicle_axle_to_rear Distance from rear axle to rear of vehicle
     * @param vehicle_width Width of the vehicle
     * @param vehicle_enable_reverse Whether reverse movement is enabled
     */
    void setVehicleParameters(
        float vehicle_max_steering_angle,
        float vehicle_steer_precision,
        float vehicle_wheelbase,
        float vehicle_axle_to_front,
        float vehicle_axle_to_rear,
        float vehicle_width,
        bool vehicle_enable_reverse);

    /**
     * @brief Set cost weights for path planning
     * @param cost_weight_reverse Weight for reverse movement cost
     * @param cost_weight_direction_change Weight for direction change cost
     * @param cost_weight_steering_angle Weight for steering angle cost
     * @param cost_weight_steering_rate Weight for steering rate cost
     * @param cost_weight_hybrid Weight for hybrid cost
     */
    void setCostWeights(
        float cost_weight_reverse,
        float cost_weight_direction_change,
        float cost_weight_steering_angle,
        float cost_weight_steering_rate,
        float cost_weight_hybrid);

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
    bool setMapParameters(
        float map_resolution,
        float map_x_min,
        float map_y_min,
        float map_x_max,
        float map_y_max,
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>&& map_obstacles);

    /**
     * @brief Plan path from start to goal state
     * @param start_state Start state [x, y, yaw]
     * @param goal_state Goal state [x, y, yaw]
     * @return Vector of waypoints forming the path
     */
    std::vector<std::array<float, 3>> plan(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state);

private:

    /* Hybrid A Star Parameters */
    // portion of a circle.
    std::size_t config_num_direction_division_;         ///< Number of direction divisions
    float config_rotation_degree_step_;                 ///< Rotation step in degrees
    float config_rotation_degree_step_negative_;        ///< Negative rotation step in degrees
    float config_rotation_radian_step_;                 ///< Rotation step in radians
    float config_rotation_radian_step_negative_;        ///< Negative rotation step in radians
    float config_step_length_;                          ///< Step length for each movement

    /* Vehicle parameters */
    float vehicle_max_steering_angle_;                  ///< Maximum steering angle in radians
    float vehicle_steer_precision_;                     ///< Steering precision
    float vehicle_wheelbase_;                           ///< Wheelbase of the vehicle
    float vehicle_axle_to_front_;                       ///< Distance from rear axle to front of vehicle
    float vehicle_axle_to_rear_;                        ///< Distance from rear axle to rear of vehicle
    float vehicle_width_;                               ///< Width of the vehicle
    bool vehicle_enable_reverse_;                       ///< Whether reverse movement is enabled

    // Cost weights
    float cost_weight_reverse_;                         ///< Weight for reverse movement cost
    float cost_weight_direction_change_;                ///< Weight for direction change cost
    float cost_weight_steering_angle_;                  ///< Weight for steering angle cost
    float cost_weight_steering_rate_;                   ///< Weight for steering rate cost
    float cost_weight_hybrid_;                          ///< Weight for hybrid cost

    /* Map Parameters */
    float map_resolution_;                              ///< Map resolution in meters per cell
    float map_x_min_;                                   ///< Minimum x coordinate of map
    float map_y_min_;                                   ///< Minimum y coordinate of map
    float map_x_max_;                                   ///< Maximum x coordinate of map
    float map_y_max_;                                   ///< Maximum y coordinate of map
    std::size_t map_grid_rows_;                         ///< Number of rows in grid
    std::size_t map_grid_cols_;                         ///< Number of columns in grid
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map_obstacles_;  ///< Obstacle map

    std::unique_ptr<KDTree> obstacle_tree_;             ///< KD-tree for obstacle lookup

    // Motion commands
    // {{yaw, direction}, {}, {}}
    std::vector<MotionCommand> motion_commands_;        ///< Available motion commands

    std::vector<Node> node_pool_;                       ///< Pool of nodes for path planning
    std::vector<Node> dubins_path_node_pool_;           ///< Pool of nodes for Dubins path

    AStarPlanner a_star_planner_;                       ///< A* planner for heuristic calculation

    /**
     * @brief Get unique node identifier based on position and orientation
     * @param x X coordinate
     * @param y Y coordinate
     * @param yaw Yaw angle in radians
     * @return Unique node ID
     */
    std::size_t getNodeID(float x, float y, float yaw);

    /**
     * @brief Check if position and orientation are within map bounds
     * @param x X coordinate
     * @param y Y coordinate
     * @param yaw Yaw angle in radians
     * @return True if position is valid
     */
    bool checkGeometry(float x, float y, float yaw) const;

    /**
     * @brief Check if vehicle at position collides with obstacles
     * @param x X coordinate
     * @param y Y coordinate
     * @param yaw Yaw angle in radians
     * @return True if collision detected
     */
    bool checkCollision(float x, float y, float yaw) const;

    /**
     * @brief Normalize angle to [0, 2π] range
     * @param angle Angle in radians
     * @return Normalized angle
     */
    inline float getPositiveNormalizedRadianAngle(const float& angle);

    /**
     * @brief Normalize angle to [-π, π] range
     * @param angle Angle in radians
     * @return Normalized angle
     */
    inline float getSymmetricNormalizedRadianAngle(const float& angle);

    /**
     * @brief Check if two nodes are equal based on position and orientation
     * @param node_a First node
     * @param node_b Second node
     * @return True if nodes are equal
     */
    inline bool isNodeEqual(const Node& node_a, const Node& node_b);

    /**
     * @brief Convert grid indices to world coordinates
     * @param row Grid row index
     * @param col Grid column index
     * @return Tuple of (x, y) world coordinates
     */
    std::tuple<float, float> calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const;

    /**
     * @brief Convert world coordinates to grid indices
     * @param x World x coordinate
     * @param y World y coordinate
     * @return Tuple of (row, col) grid indices
     */
    std::tuple<std::size_t, std::size_t> calculateGridIndexFromCoordinate(float x, float y) const;

    /**
     * @brief Initialize motion commands based on vehicle parameters
     */
    void initializeMotionCommands();

    /**
     * @brief Get expanded state based on motion command
     * @param current_node Current node
     * @param motion_command Motion command to apply
     * @return Array with new state [x, y, yaw]
     */
    std::array<float, 3> getExpandedState(const Node& current_node, const MotionCommand& motion_command);

    /**
     * @brief Get Dubins shot path from start to goal
     * @param start_node Start node
     * @param goal_node Goal node
     * @return Pointer to goal node if Dubins path found, nullptr otherwise
     */
    Node* getDubinsShot(Node* start_node, Node* goal_node);

    /**
     * @brief Calculate movement cost between nodes
     * @param prev_node Previous node
     * @param motion_command Motion command used to reach new node
     * @return Movement cost
     */
    float calculateGCost(const Node& prev_node, const MotionCommand& motion_command) const;

    /**
     * @brief Calculate heuristic cost to goal using A* and Dubins path
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic cost
     */
    float calculateHuristicCost(const Node& current_node, const Node& goal_node);

    /**
     * @brief Extract waypoints from path
     * @param end_node_ptr Pointer to end node
     * @return Vector of waypoints as [x, y, yaw] coordinates
     */
    std::vector<std::array<float, 3>> getPathWaypoints(const Node* end_node_ptr);

    /**
     * @brief Find path between start and goal states
     * @param start_state Start state [x, y, yaw]
     * @param goal_state Goal state [x, y, yaw]
     * @return Pointer to goal node if path found, nullptr otherwise
     */
    Node* findPath(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state);
};

#endif // __HYBRID_A_STAR_PLANNER_H__