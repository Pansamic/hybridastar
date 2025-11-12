#include <iostream>
#include <filesystem>
#include <random>
#include <Eigen/Core>
#include <data_saver.hpp>
#include <hybridastar/a_star_planner.h>

static const float kMapResolution = 0.1;
static const float kMapXMin = -10.0;
static const float kMapYMin = -10.0;
static const float kMapXMax = 10.0;
static const float kMapYMax = 10.0;

// Helper function to check if a coordinate is in free space
bool isFreeCoordinate(const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &map,
                      float x, float y, float resolution, float x_min, float y_min)
{
    // Convert coordinates to grid indices
    std::size_t row = static_cast<std::size_t>(std::floor((x - x_min) / resolution));
    std::size_t col = static_cast<std::size_t>(std::floor((y - y_min) / resolution));

    // Check bounds
    if (row >= map.rows() || col >= map.cols())
    {
        return false; // Out of bounds
    }

    // Check if the cell is free (0 means free, 1 means obstacle)
    return map(row, col) == 0;
}

// Extract path from A* result node by following parent pointers
std::vector<std::array<float, 3>> extractPath(const AStarPlanner::Node *end_node, float resolution, float x_min, float y_min)
{
    std::vector<std::array<float, 3>> path;
    const AStarPlanner::Node *current = end_node;

    while (current != nullptr)
    {
        // Convert grid indices back to coordinates
        float x = (current->row + 0.5f) * resolution + x_min; // Adding 0.5 for cell center
        float y = (current->col + 0.5f) * resolution + y_min; // Adding 0.5 for cell center

        // For A* path, theta is not meaningful, so set to 0
        path.push_back({x, y, 0.0f});
        current = current->parent;
    }

    // Reverse the path since we built it backwards
    std::reverse(path.begin(), path.end());
    return path;
}

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> generateMap1()
{
    std::size_t map_row = static_cast<std::size_t>((kMapYMax - kMapYMin) / kMapResolution);
    std::size_t map_col = static_cast<std::size_t>((kMapXMax - kMapXMin) / kMapResolution);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map(map_row, map_col);
    map.setZero();

    for (std::size_t i = map_row / 4; i < map_row * 3 / 4; i++)
    {
        map(i, map_col / 2) = 1;
    }
    for (std::size_t i = map_col / 4; i < map_col * 3 / 4; i++)
    {
        map(map_row / 2, i) = 1;
    }

    return map;
}

int main()
{
    // Generate the map
    auto map = generateMap1();

    // Choose start and goal positions that avoid known obstacles
    std::array<float, 2> start_pos = {-8.0f, -8.0f}; // Starting from a known free position
    std::array<float, 2> goal_pos = {8.0f, 8.0f};    // Goal at another known free position

    // Initialize A* planner
    AStarPlanner planner;
    auto map_copy = map;
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(map_copy));

    // Run A* planning
    auto result = planner.findPath(start_pos, goal_pos);

    if (result.has_value())
    {
        std::cout << "A* path found!" << std::endl;

        // Extract the full path
        auto path = extractPath(result.value(), kMapResolution, kMapXMin, kMapYMin);
        std::cout << "Path length: " << path.size() << " waypoints" << std::endl;

        // Save map and path data to binary file
        MapAndPathData data;
        data.algorithm_type = AlgorithmType::ASTAR;
        data.resolution = kMapResolution;
        data.x_min = kMapXMin;
        data.x_max = kMapXMax;
        data.y_min = kMapYMin;
        data.y_max = kMapYMax;
        data.rows = static_cast<std::size_t>((kMapYMax - kMapYMin) / kMapResolution);
        data.cols = static_cast<std::size_t>((kMapXMax - kMapXMin) / kMapResolution);

        // Get map data
        data.map_data.assign(map.data(), map.data() + map.size());
        data.path = path;

        // Set vehicle parameters (not used for A*, but stored for consistency)
        data.vehicle_wheelbase = 1.0f;
        data.vehicle_axle_to_front = 0.5f;
        data.vehicle_axle_to_rear = 0.5f;
        data.vehicle_width = 0.7f;

        // Save to binary file
        std::string filename = "temp/test_a_star.bin";
        std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
        if (DataSaver::saveMapAndPath(filename, data))
        {
            std::cout << "A* data saved to " << filename << std::endl;
        }
        else
        {
            std::cout << "Failed to save A* data to " << filename << std::endl;
        }
    }
    else
    {
        std::cout << "A* path not found!" << std::endl;
    }

    return 0;
}