#include <iostream>
#include <fstream>
#include <filesystem>
#include <hybridastar/hybrid_a_star_planner.h>
#include <data_saver.hpp>

static const std::size_t kNumDirectionDivision = 36;
static const float kStepLength = 0.2;

static const float kVehicleMaxSteeringAngle = M_PI_4;
static const float kVehicleSteerPrecision = 4.0;
static const float kVehicleWheelbase = 0.5;
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

static const float kElevationThreshold = 0.0;

int main()
{
    HybridAStarPlanner planner;
    planner.setConfigParameters(kNumDirectionDivision, kStepLength);
    planner.setCostWeights(kCostWeightReverse, kCostWeightDirectionChange, kCostWeightSteeringAngle, kCostWeightSteeringRate, kCostWeightHybrid);
    planner.setVehicleParameters(kVehicleMaxSteeringAngle, kVehicleSteerPrecision, kVehicleWheelbase, kVehicleAxleToFront, kVehicleAxleToRear, kVehicleWidth, kVehicleEnableReverse);

    // Read elevation map from binary file
    std::ifstream file("assets/elevation_map.bin", std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open elevation map file" << std::endl;
        return -1;
    }

    uint32_t rows, cols;
    file.read(reinterpret_cast<char *>(&rows), sizeof(uint32_t));
    file.read(reinterpret_cast<char *>(&cols), sizeof(uint32_t));

    Eigen::MatrixXf elevation_map(rows, cols);
    file.read(reinterpret_cast<char *>(elevation_map.data()), rows * cols * sizeof(float));
    file.close();

    float map_x_min = -(static_cast<float>(rows) * kMapResolution / 2.0f);
    float map_x_max =  (static_cast<float>(rows) * kMapResolution / 2.0f);
    float map_y_min = -(static_cast<float>(cols) * kMapResolution / 2.0f);
    float map_y_max =  (static_cast<float>(cols) * kMapResolution / 2.0f);

    auto nan_filled_map = elevation_map.unaryExpr([](float x) { return std::isnan(x) ? std::numeric_limits<float>::max() : x; });

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map = (nan_filled_map.array() > kElevationThreshold).cast<uint8_t>();
    auto export_map = map;

    if (!planner.setMapParameters(kMapResolution, map_x_min, map_y_min, map_x_max, map_y_max, std::move(map)))
    {
        std::cout << "Failed to set map parameters" << std::endl;
        return -1;
    }

    auto path = planner.plan({{-2.0, -1.0, M_PI / 6.0}}, {{2.3, 1.0, M_PI / 6.0}});

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
        return -1;
    }

    // Save map and path data to binary file
    MapAndPathData data;
    data.algorithm_type = AlgorithmType::HYBRID_ASTAR;
    data.resolution = kMapResolution;
    data.x_min = map_x_min;
    data.x_max = map_x_max;
    data.y_min = map_y_min;
    data.y_max = map_y_max;
    data.rows = rows;
    data.cols = cols;

    // Get map data from the planner
    // Since the map is not directly accessible, we'll use the generated map
    data.map_data.assign(export_map.data(), export_map.data() + export_map.size());

    data.path = path;

    // Set vehicle parameters
    data.vehicle_wheelbase = kVehicleWheelbase;
    data.vehicle_axle_to_front = kVehicleAxleToFront;
    data.vehicle_axle_to_rear = kVehicleAxleToRear;
    data.vehicle_width = kVehicleWidth;

    // Save to binary file
    std::string filename = "temp/test_real_map.bin";

    std::filesystem::create_directories(std::filesystem::path(filename).parent_path());

    if (DataSaver::saveMapAndPath(filename, data))
    {
        std::cout << "Data saved to " << filename << std::endl;
    }
    else
    {
        std::cout << "Failed to save data to " << filename << std::endl;
    }

    return 0;
}