
#ifndef __HYBRID_A_STAR_PLANNER_H__
#define __HYBRID_A_STAR_PLANNER_H__ 

#include <cstdint>
#include <cmath>
#include <expected>
#include <array>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <hybridastar/kdtree.hpp>
#include <hybridastar/a_star_planner.h>

class HybridAStarPlanner final
{
public:
    struct MotionCommand
    {
        float rotation;
        int direction;
        MotionCommand()
        {
            this->rotation = 0.0;
            this->direction = 1;
        }
        MotionCommand(float rotation, int direction)
        {
            this->rotation = rotation;
            this->direction = direction;
        }
        void operator=(const MotionCommand& motion_command)
        {
            this->rotation = motion_command.rotation;
            this->direction = motion_command.direction;
        }
    };

    struct Node
    {
        float x;
        float y;
        float yaw;
        float cost_g;
        float cost_h;
        uint8_t status; // 0: uninitialized, 1: open, 2: closed
        const MotionCommand* motion;
        const Node* parent;

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
        bool isUninitialized() const {return this->status == 0;}
        bool isOpen() const {return this->status == 1;}
        bool isClosed() const {return this->status == 2;}
        void setOpen() {this->status = 1;}
        void setClosed() {this->status = 2;}
        bool operator()(const Node* lhs, const Node* rhs) const
        {
            return (lhs->cost_g + lhs->cost_h) > (rhs->cost_g + rhs->cost_h);
        }
    };

    enum class ErrorCode
    {
        SUCCESS = 0,
        INVALID_ARGUMENT,
        INVALID_GEOMETRY,
        COLLISION,
        NO_PATH
    };

    explicit HybridAStarPlanner() = default;
    ~HybridAStarPlanner() = default;
    
    void setConfigParameters(std::size_t num_direction_division, float step_length);
    void setVehicleParameters(
        float vehicle_max_steering_angle,
        float vehicle_steer_precision,
        float vehicle_wheelbase,
        float vehicle_axle_to_front,
        float vehicle_axle_to_rear,
        float vehicle_width,
        bool vehicle_enable_reverse);
    void setCostWeights(
        float cost_weight_reverse,
        float cost_weight_direction_change,
        float cost_weight_steering_angle,
        float cost_weight_steering_rate,
        float cost_weight_hybrid);
    ErrorCode setMapParameters(
        float map_resolution,
        float map_x_min,
        float map_y_min,
        float map_x_max,
        float map_y_max,
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>&& map_obstacles);
    std::vector<std::array<float, 3>> plan(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state);
private:

    /* Hybrid A Star Parameters */
    // portion of a circle.
    std::size_t config_num_direction_division_;
    float config_rotation_degree_step_;
    float config_rotation_degree_step_negative_;
    float config_rotation_radian_step_;
    float config_rotation_radian_step_negative_;
    float config_step_length_;

    /* Vehicle parameters */
    float vehicle_max_steering_angle_;
    float vehicle_steer_precision_;
    float vehicle_wheelbase_;
    float vehicle_axle_to_front_;
    float vehicle_axle_to_rear_;
    float vehicle_width_;
    bool vehicle_enable_reverse_;

    // Cost weights
    float cost_weight_reverse_;
    float cost_weight_direction_change_;
    float cost_weight_steering_angle_;
    float cost_weight_steering_rate_;
    float cost_weight_hybrid_;

    /* Map Parameters */
    float map_resolution_;
    float map_x_min_;
    float map_y_min_;
    float map_x_max_;
    float map_y_max_;
    std::size_t map_grid_rows_;
    std::size_t map_grid_cols_;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map_obstacles_;

    std::unique_ptr<KDTree> obstacle_tree_;

    // Motion commands
    // {{yaw, direction}, {}, {}}
    std::vector<MotionCommand> motion_commands_;

    std::vector<Node> node_pool_;
    std::vector<Node> dubins_path_node_pool_;

    AStarPlanner a_star_planner_;

    std::size_t getNodeID(float x, float y, float yaw);
    bool checkGeometry(float x, float y, float yaw) const;
    bool checkCollision(float x, float y, float yaw) const;
    inline float getPositiveNormalizedRadianAngle(const float& angle);
    inline bool isNodeXYTEqual(const Node& node_a, const Node& node_b);
    std::tuple<float, float> calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const;
    std::tuple<std::size_t, std::size_t> calculateGridIndexFromCoordinate(float x, float y) const;

    void initializeMotionCommands();
    std::expected<std::array<float, 3>, ErrorCode> getExpandedState(const Node& current_node, const MotionCommand& motion_command);
    std::expected<Node*, ErrorCode> getDubinsShot(Node* start_node, Node* goal_node);
    float calculateGCost(const Node& prev_node, const MotionCommand& motion_command) const;
    float calculateHuristicCost(const Node& current_node, const Node& goal_node);
    std::vector<std::array<float, 3>> getPathWaypoints(const Node* end_node_ptr);
    std::expected<const Node*, ErrorCode> findPath(const std::array<float, 3>& start_state, const std::array<float, 3>& goal_state);
};

#endif // __HYBRID_A_STAR_PLANNER_H__