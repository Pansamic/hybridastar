#include <iostream>
#include <hybrid_a_star_planner.h>

static const std::size_t kNumDirectionDivision = 72;
static const float kStepLength = 0.2;

static const float kVehicleMaxSteeringAngle = 0.6;
static const float kVehicleSteerPrecision = 10;
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
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(generateSimpleMap()));
    planner.setVehicleParameters(kVehicleMaxSteeringAngle, kVehicleSteerPrecision, kVehicleWheelbase, kVehicleAxleToFront, kVehicleAxleToRear, kVehicleWidth, kVehicleEnableReverse);
    auto path = planner.plan({{0.0, 0.0, 0.0}}, {{3.0, 3.0, M_PI_2}});
    if (!path.empty())
    {
        std::cout << "Path found! Length: " << path.size() << " waypoints" << std::endl;
        // Print first few and last few waypoints
        for (int i = 0; i < std::min(static_cast<size_t>(5), path.size()); ++i) {
            std::cout << "x=" << path[i][0] << ";y=" << path[i][1] << ";z=" << path[i][2] << std::endl;
        }
        if (path.size() > 5) {
            std::cout << "..." << std::endl;
            for (int i = std::max(static_cast<int>(path.size())-5, 0); i < static_cast<int>(path.size()); ++i) {
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