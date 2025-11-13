#include <iostream>
#include <filesystem>
#include <random>
#include <Eigen/Core>
#include <data_saver.hpp>
#include <hybridastar/a_star_planner.h>

static const float kMapResolution = 0.1;
static const float kMapXMin = -7.2;
static const float kMapYMin = -8.3;
static const float kMapXMax = 4.15;
static const float kMapYMax = 5.4;

inline std::tuple<std::size_t, std::size_t> calculateGridIndexFromCoordinate(float x, float y)
{
    std::size_t row = static_cast<std::size_t>(std::floor((kMapXMax - x) / kMapResolution));
    std::size_t col = static_cast<std::size_t>(std::floor((kMapYMax - y) / kMapResolution));
    return std::make_tuple(row, col);
}

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> generateMap1()
{
    std::size_t map_row = static_cast<std::size_t>(std::lround((kMapXMax - kMapXMin) / kMapResolution));
    std::size_t map_col = static_cast<std::size_t>(std::lround((kMapYMax - kMapYMin) / kMapResolution));
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map(map_row, map_col);
    map.setZero();

    auto [zero_row, zero_col] = calculateGridIndexFromCoordinate(0.0f, 0.0f);

    for (std::size_t i = map_row / 4; i < map_row * 3 / 4; i++)
    {
        map(i, zero_col) = 1;
    }
    for (std::size_t i = map_col / 4; i < map_col * 3 / 4; i++)
    {
        map(zero_row, i) = 1;
    }

    return map;
}

int main()
{
    // Generate the map
    auto map = generateMap1();

    // Choose start and goal positions that avoid known obstacles
    std::array<float, 2> start_pos = {-0.2f, -0.2f}; // Starting from a known free position
    std::array<float, 2> goal_pos = {0.2f, 0.2f};    // Goal at another known free position

    // Initialize A* planner
    AStarPlanner planner;
    auto map_copy = map;
    if (!planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(map_copy)))
    {
        std::cout << "Failed to set map parameters for A* planner" << std::endl;
        return -1;
    }

    // Run A* planning
    auto path = planner.plan(start_pos, goal_pos);

    if (!path.empty())
    {
        std::cout << "A* path found!" << std::endl;
        std::cout << "Path length: " << path.size() << " waypoints" << std::endl;

        std::vector<std::array<float, 3>> waypoints;

        for (const auto &pathpoint : path)
        {
            waypoints.emplace_back(std::array<float, 3>{pathpoint[0], pathpoint[1], 0.0f});
        }

        // Save map and path data to binary file
        MapAndPathData data;
        data.algorithm_type = AlgorithmType::ASTAR;
        data.resolution = kMapResolution;
        data.x_min = kMapXMin;
        data.x_max = kMapXMax;
        data.y_min = kMapYMin;
        data.y_max = kMapYMax;
        data.rows = map.rows();
        data.cols = map.cols();

        // Get map data
        data.map_data.assign(map.data(), map.data() + map.size() * sizeof(uint8_t));
        data.path = waypoints;

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