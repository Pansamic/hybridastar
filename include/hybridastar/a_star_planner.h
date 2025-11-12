
#ifndef __A_STAR_PLANNER_H__
#define __A_STAR_PLANNER_H__ 

#include <cstdint>
#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include <hybridastar/kdtree.hpp>
#include <Eigen/Core>

class AStarPlanner final
{
public:
    struct Node
    {
        std::size_t row;
        std::size_t col;
        float cost_g;
        float cost_h;
        uint8_t status;
        const Node* parent;

        Node()
        {
            this->row = 0;
            this->col = 0;
            this->cost_g = 0.0;
            this->cost_h = 0.0;
            this->status = 0; // uninitialized status
            this->parent = nullptr;
        }
        Node(std::size_t row, std::size_t col)
        {
            this->row = row;
            this->col = col;
            this->cost_g = 0.0f;
            this->cost_h = 0.0f;
            this->status = 0; // uninitialized status
            this->parent = nullptr;
        }
        bool isOpen() const
        {
            return this->status == 1;
        }
        bool isClosed() const
        {
            return this->status == 2;
        }
        bool isUninitialized() const
        {
            return this->status == 0;
        }
        void setOpen()
        {
            this->status = 1;
        }
        void setClosed()
        {
            this->status = 2;
        }
        bool operator==(const Node& other)
        {
            return (this->row == other.row && this->col == other.col);
        }
        bool operator()(const Node* lhs, const Node* rhs) const
        {
            return (lhs->cost_g + lhs->cost_h) > (rhs->cost_g + rhs->cost_h);
        }
    };

    explicit AStarPlanner() = default;
    ~AStarPlanner() = default;

    void setMapParameters(float map_resolution, float map_x_min, float map_y_min, float map_x_max, float map_y_max, Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>&& map_obstacles);
    Node* findPath(const std::array<float, 2>& start_pos, const std::array<float, 2>& goal_pos);
    float getPathLength(const Node* end_node_ptr);
    std::vector<std::array<float, 2>> getPathWaypoints(const Node* end_node_ptr);
    std::vector<std::array<float, 2>> plan(const std::array<float, 2>& start_pos, const std::array<float, 2>& goal_pos);
private:
    /* Map Parameters */
    float map_resolution_;
    float map_x_min_;
    float map_y_min_;
    float map_x_max_;
    float map_y_max_;
    std::size_t map_grid_rows_;
    std::size_t map_grid_cols_;

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map_obstacles_;

    std::vector<Node> node_pool_;

    static const std::array<std::array<int8_t, 2>, 8> motion_commands_;

    inline std::size_t getNodeID(std::size_t row, std::size_t col) const;
    inline bool checkGeometry(std::size_t row, std::size_t col) const;
    inline bool checkCollision(std::size_t row, std::size_t col) const;
    inline std::tuple<float, float> calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const;
    inline std::tuple<std::size_t, std::size_t> calculateGridIndexFromCoordinate(float x, float y) const;
    
    std::array<std::size_t, 2> getExpandedPosition(const Node& current_node, const std::array<int8_t, 2>& motion_command);
    float calculateGCost(const Node& prev_node, const std::array<int8_t, 2>& motion_command) const;
    float calculateHuristicCost(const Node& current_node, const Node& goal_node) const;
};

#endif //__A_STAR_PLANNER_H__