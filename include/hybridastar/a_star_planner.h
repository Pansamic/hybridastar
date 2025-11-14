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
    /**
     * @brief Node representation for A* algorithm
     */
    struct Node
    {
        std::size_t row;           ///< Grid row index
        std::size_t col;           ///< Grid column index
        float cost_g;              ///< Actual cost from start node
        float cost_h;              ///< Heuristic cost to goal node
        uint8_t status;            ///< Node status (0: uninitialized, 1: open, 2: closed)
        const Node* parent;        ///< Pointer to parent node

        /**
         * @brief Default constructor
         */
        Node()
        {
            this->row = 0;
            this->col = 0;
            this->cost_g = 0.0;
            this->cost_h = 0.0;
            this->status = 0; // uninitialized status
            this->parent = nullptr;
        }
        
        /**
         * @brief Constructor with position
         * @param row Grid row index
         * @param col Grid column index
         */
        Node(std::size_t row, std::size_t col)
        {
            this->row = row;
            this->col = col;
            this->cost_g = 0.0f;
            this->cost_h = 0.0f;
            this->status = 0; // uninitialized status
            this->parent = nullptr;
        }
        
        /**
         * @brief Check if node is in open set
         * @return True if node is in open set
         */
        bool isOpen() const
        {
            return this->status == 1;
        }
        
        /**
         * @brief Check if node is in closed set
         * @return True if node is in closed set
         */
        bool isClosed() const
        {
            return this->status == 2;
        }
        
        /**
         * @brief Check if node is uninitialized
         * @return True if node is uninitialized
         */
        bool isUninitialized() const
        {
            return this->status == 0;
        }
        
        /**
         * @brief Set node status to open
         */
        void setOpen()
        {
            this->status = 1;
        }
        
        /**
         * @brief Set node status to closed
         */
        void setClosed()
        {
            this->status = 2;
        }
        
        /**
         * @brief Equality operator for nodes
         * @param other Node to compare with
         * @return True if nodes have same position
         */
        bool operator==(const Node& other)
        {
            return (this->row == other.row && this->col == other.col);
        }
        
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
    explicit AStarPlanner() = default;
    
    /**
     * @brief Destructor
     */
    ~AStarPlanner() = default;

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
    bool setMapParameters(float map_resolution, float map_x_min, float map_y_min, float map_x_max, float map_y_max, Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>&& map_obstacles);
    
    /**
     * @brief Find path between start and goal positions
     * @param start_pos Start position [x, y]
     * @param goal_pos Goal position [x, y]
     * @return Pointer to goal node if path found, nullptr otherwise
     */
    Node* findPath(const std::array<float, 2>& start_pos, const std::array<float, 2>& goal_pos);
    
    /**
     * @brief Get path length from start to end node
     * @param end_node_ptr Pointer to end node
     * @return Path length, or -1 if node is null
     */
    float getPathLength(const Node* end_node_ptr);
    
    /**
     * @brief Extract waypoints from path
     * @param end_node_ptr Pointer to end node
     * @return Vector of waypoints as [x, y] coordinates
     */
    std::vector<std::array<float, 2>> getPathWaypoints(const Node* end_node_ptr);
    
    /**
     * @brief Plan path from start to goal
     * @param start_pos Start position [x, y]
     * @param goal_pos Goal position [x, y]
     * @return Vector of waypoints forming the path
     */
    std::vector<std::array<float, 2>> plan(const std::array<float, 2>& start_pos, const std::array<float, 2>& goal_pos);
    
private:
    /* Map Parameters */
    float map_resolution_;          ///< Map resolution in meters per cell
    float map_x_min_;               ///< Minimum x coordinate of map
    float map_y_min_;               ///< Minimum y coordinate of map
    float map_x_max_;               ///< Maximum x coordinate of map
    float map_y_max_;               ///< Maximum y coordinate of map
    std::size_t map_grid_rows_;     ///< Number of rows in grid
    std::size_t map_grid_cols_;     ///< Number of columns in grid

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map_obstacles_;  ///< Obstacle map

    std::vector<Node> node_pool_;   ///< Pool of nodes for path planning

    static const std::array<std::array<int8_t, 2>, 8> motion_commands_;     ///< Possible movement directions

    /**
     * @brief Get unique node identifier based on position
     * @param row Grid row index
     * @param col Grid column index
     * @return Unique node ID
     */
    inline std::size_t getNodeID(std::size_t row, std::size_t col) const;
    
    /**
     * @brief Check if grid position is within map bounds
     * @param row Grid row index
     * @param col Grid column index
     * @return True if position is valid
     */
    inline bool checkGeometry(std::size_t row, std::size_t col) const;
    
    /**
     * @brief Check if grid cell contains obstacle
     * @param row Grid row index
     * @param col Grid column index
     * @return True if cell contains obstacle
     */
    inline bool checkCollision(std::size_t row, std::size_t col) const;
    
    /**
     * @brief Convert grid indices to world coordinates
     * @param row Grid row index
     * @param col Grid column index
     * @return Tuple of (x, y) world coordinates
     */
    inline std::tuple<float, float> calculateCoordinateFromGridIndex(std::size_t row, std::size_t col) const;
    
    /**
     * @brief Convert world coordinates to grid indices
     * @param x World x coordinate
     * @param y World y coordinate
     * @return Tuple of (row, col) grid indices
     */
    inline std::tuple<std::size_t, std::size_t> calculateGridIndexFromCoordinate(float x, float y) const;
    
    /**
     * @brief Get expanded position based on motion command
     * @param current_node Current node
     * @param motion_command Motion command [dx, dy]
     * @return Array with new position [row, col]
     */
    std::array<std::size_t, 2> getExpandedPosition(const Node& current_node, const std::array<int8_t, 2>& motion_command);
    
    /**
     * @brief Calculate movement cost between nodes
     * @param prev_node Previous node
     * @param motion_command Motion command used to reach new node
     * @return Movement cost
     */
    float calculateGCost(const Node& prev_node, const std::array<int8_t, 2>& motion_command) const;
    
    /**
     * @brief Calculate heuristic cost to goal
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic cost
     */
    float calculateHuristicCost(const Node& current_node, const Node& goal_node) const;
};

#endif //__A_STAR_PLANNER_H__