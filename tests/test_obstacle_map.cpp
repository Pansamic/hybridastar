#include <iostream>
#include <filesystem>
#include <hybridastar/hybrid_a_star_planner.h>
#include <data_saver.hpp>

static const std::size_t kNumDirectionDivision = 36;
static const float kStepLength = 0.2;

static const float kVehicleMaxSteeringAngle = M_PI_4;
static const float kVehicleSteerPrecision = 4.0;
static const float kVehicleWheelbase = 1;
static const float kVehicleAxleToFront = 0.5;
static const float kVehicleAxleToRear = 0.5;
static const float kVehicleWidth = 0.7;
static const bool kVehicleEnableReverse = false;

static const float kCostWeightReverse = 10.0;
static const float kCostWeightDirectionChange = 150.0;
static const float kCostWeightSteeringAngle = 1.0;
static const float kCostWeightSteeringRate = 5.0;
static const float kCostWeightHybrid = 50.0;

static const float kMapResolution = 0.1;
static const float kMapXMin = -8.0;
static const float kMapYMin = -10.0;
static const float kMapXMax = 8.0;
static const float kMapYMax = 10.0;

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
    HybridAStarPlanner planner;
    planner.setConfigParameters(kNumDirectionDivision, kStepLength);
    planner.setCostWeights(kCostWeightReverse, kCostWeightDirectionChange, kCostWeightSteeringAngle, kCostWeightSteeringRate, kCostWeightHybrid);

    auto map = generateMap1();
    auto map_copy = map;
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(map_copy));
    planner.setVehicleParameters(kVehicleMaxSteeringAngle, kVehicleSteerPrecision, kVehicleWheelbase, kVehicleAxleToFront, kVehicleAxleToRear, kVehicleWidth, kVehicleEnableReverse);
    auto path = planner.plan({{-1.0, -1.0, 0.0}}, {{1.0, 7.5, M_PI}});

    // Save map and path data to binary file
    MapAndPathData data;
    data.algorithm_type = AlgorithmType::HYBRID_ASTAR;
    data.resolution = kMapResolution;
    data.x_min = kMapXMin;
    data.x_max = kMapXMax;
    data.y_min = kMapYMin;
    data.y_max = kMapYMax;
    data.rows = map.rows();
    data.cols = map.cols();

    // Get map data from the planner
    // Since the map is not directly accessible, we'll use the generated map
    data.map_data.assign(map.data(), map.data() + map.size());

    data.path = path;

    // Set vehicle parameters
    data.vehicle_wheelbase = kVehicleWheelbase;
    data.vehicle_axle_to_front = kVehicleAxleToFront;
    data.vehicle_axle_to_rear = kVehicleAxleToRear;
    data.vehicle_width = kVehicleWidth;

    // Save to binary file
    std::string filename = "temp/test_obstacle_map.bin";

    std::filesystem::create_directories(std::filesystem::path(filename).parent_path());

    if (DataSaver::saveMapAndPath(filename, data))
    {
        std::cout << "Data saved to " << filename << std::endl;
    }
    else
    {
        std::cout << "Failed to save data to " << filename << std::endl;
    }

    if (!path.empty())
    {
        std::cout << "Path found!" << std::endl;
        for (auto &waypoint : path)
        {
            std::cout << "x=" << waypoint[0] << ";y=" << waypoint[1] << ";z=" << waypoint[2] << std::endl;
        }
    }
    else
    {
        std::cout << "Path not found!" << std::endl;
    }
    return 0;
}