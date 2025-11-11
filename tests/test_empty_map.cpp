#include <iostream>
#include <hybrid_a_star_planner.h>
#include <data_saver.h>

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
static const float kMapXMin = -10.0;
static const float kMapYMin = -10.0;
static const float kMapXMax = 10.0;
static const float kMapYMax = 10.0;

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> generateSimpleMap()
{
    std::size_t map_row = static_cast<std::size_t>((kMapYMax - kMapYMin) / kMapResolution);
    std::size_t map_col = static_cast<std::size_t>((kMapXMax - kMapXMin) / kMapResolution);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map(map_row, map_col);
    map.setZero(); // completely free space

    return map;
}

int main()
{
    HybridAStarPlanner planner;
    planner.setConfigParameters(kNumDirectionDivision, kStepLength);
    planner.setCostWeights(kCostWeightReverse, kCostWeightDirectionChange, kCostWeightSteeringAngle, kCostWeightSteeringRate, kCostWeightHybrid);

    auto map = generateSimpleMap();
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(map));
    planner.setVehicleParameters(kVehicleMaxSteeringAngle, kVehicleSteerPrecision, kVehicleWheelbase, kVehicleAxleToFront, kVehicleAxleToRear, kVehicleWidth, kVehicleEnableReverse);
    auto path = planner.plan({{-8.0, -8.0, 0.0}}, {{8.0, 8.0, M_PI_2}});

    // Save map and path data to binary file
    MapAndPathData data;
    data.resolution = kMapResolution;
    data.x_min = kMapXMin;
    data.x_max = kMapXMax;
    data.y_min = kMapYMin;
    data.y_max = kMapYMax;
    data.rows = static_cast<std::size_t>((kMapYMax - kMapYMin) / kMapResolution);
    data.cols = static_cast<std::size_t>((kMapXMax - kMapXMin) / kMapResolution);

    // Get map data from the planner
    // Since the map is not directly accessible, we'll use the generated map
    auto generated_map = generateSimpleMap();
    data.map_data.assign(generated_map.data(), generated_map.data() + generated_map.size());

    data.path = path;

    // Set vehicle parameters
    data.vehicle_wheelbase = kVehicleWheelbase;
    data.vehicle_axle_to_front = kVehicleAxleToFront;
    data.vehicle_axle_to_rear = kVehicleAxleToRear;
    data.vehicle_width = kVehicleWidth;

    // Save to binary file
    std::string filename = "temp/test_empty_map.bin";
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
        std::cout << "Path found! Length: " << path.size() << " waypoints" << std::endl;
        // Print first few and last few waypoints
        for (int i = 0; i < std::min(static_cast<size_t>(5), path.size()); ++i)
        {
            std::cout << "x=" << path[i][0] << ";y=" << path[i][1] << ";z=" << path[i][2] << std::endl;
        }
        if (path.size() > 5)
        {
            std::cout << "..." << std::endl;
            for (int i = std::max(static_cast<int>(path.size()) - 5, 0); i < static_cast<int>(path.size()); ++i)
            {
                std::cout << "x=" << path[i][0] << ";y=" << path[i][1] << ";z=" << path[i][2] << std::endl;
            }
        }
    }
    else
    {
        std::cout << "Path not found!" << std::endl;
    }
    return 0;
}