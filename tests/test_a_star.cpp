#include <iostream>
#include <random>
#include <Eigen/Core>
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

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> x_dist(kMapXMin, kMapXMax);
    std::uniform_real_distribution<float> y_dist(kMapYMin, kMapYMax);

    const int total_tests = 10000;
    int success_count = 0;

    std::cout << "Running " << total_tests << " A* planner tests..." << std::endl;


    // Initialize A* planner for this test
    AStarPlanner planner;
    auto map_copy = map; // Make a copy for this test
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(map_copy));

    for (int i = 0; i < total_tests; ++i)
    {
        // Generate random start and goal positions
        std::array<float, 2> start_pos, goal_pos;

        // Find valid start position (not on obstacle)
        bool valid_start = false;
        int attempts = 0;
        while (!valid_start && attempts < 100)
        { // Limit attempts to avoid infinite loop
            start_pos[0] = x_dist(gen);
            start_pos[1] = y_dist(gen);

            // Check if start position is valid (within bounds and not on obstacle)
            if (isFreeCoordinate(map, start_pos[0], start_pos[1], kMapResolution, kMapXMin, kMapYMin))
            {
                valid_start = true;
            }
            attempts++;
        }

        if (!valid_start)
        {
            continue; // Skip this test if we can't find a valid start position
        }

        // Find valid goal position (not on obstacle)
        bool valid_goal = false;
        attempts = 0;
        while (!valid_goal && attempts < 100)
        { // Limit attempts to avoid infinite loop
            goal_pos[0] = x_dist(gen);
            goal_pos[1] = y_dist(gen);

            // Check if goal position is valid (within bounds and not on obstacle)
            if (isFreeCoordinate(map, goal_pos[0], goal_pos[1], kMapResolution, kMapXMin, kMapYMin))
            {
                valid_goal = true;
            }
            attempts++;
        }

        if (!valid_goal)
        {
            continue; // Skip this test if we can't find a valid goal position
        }

        // Run A* planning
        auto result = planner.findPath(start_pos, goal_pos);

        if (result != nullptr)
        {
            success_count++;
        }

        // Print progress every 1000 tests
        if ((i + 1) % 1000 == 0)
        {
            std::cout << "Completed " << (i + 1) << "/" << total_tests
                      << " tests. Successes: " << success_count << std::endl;
        }
    }

    std::cout << "\nTest Results:" << std::endl;
    std::cout << "Total tests: " << total_tests << std::endl;
    std::cout << "Successful plans: " << success_count << std::endl;
    std::cout << "Success rate: " << (static_cast<double>(success_count) / total_tests * 100.0) << "%" << std::endl;

    return 0;
}