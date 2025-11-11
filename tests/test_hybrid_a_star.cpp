#include <iostream>
#include <hybrid_a_star_planner.h>

static const std::size_t kNumDirectionDivision = 72;
static const float kStepLength = 0.2;

static const float kVehicleMaxSteeringAngle = 0.5;
static const float kVehicleSteerPrecision = 0.1;
static const float kVehicleWheelbase = 2.0;
static const float kVehicleAxleToFront = 1.0;
static const float kVehicleAxleToRear = 1.0;
static const float kVehicleWidth = 1.0;
static const bool kVehicleEnableReverse = false;

static const float kCostWeightReverse = 1.0;
static const float kCostWeightDirectionChange = 1.0;
static const float kCostWeightSteeringAngle = 1.0;
static const float kCostWeightSteeringRate = 1.0;
static const float kCostWeightHybrid = 1.0;

static const float kMapResolution = 0.1;
static const float kMapXMin = -10.0;
static const float kMapYMin = -10.0;
static const float kMapXMax = 10.0;
static const float kMapYMax = 10.0;

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> generateMap1()
{
    std::size_t map_row = static_cast<std::size_t>((kMapYMax - kMapYMin) / kMapResolution);
    std::size_t map_col = static_cast<std::size_t>((kMapXMax - kMapXMin) / kMapResolution);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> map(map_row, map_col);
    map.setZero();

    // Add obstacles around start position (0.0, 0.0)
    std::size_t start_x = static_cast<std::size_t>((0.0 - kMapXMin) / kMapResolution);
    std::size_t start_y = static_cast<std::size_t>((0.0 - kMapYMin) / kMapResolution);
    
    // Add a small rectangular obstacle near start
    for (std::size_t i = 0; i < 5 && start_y + i < map_row; ++i) {
        for (std::size_t j = 0; j < 5 && start_x + j < map_col; ++j) {
            if (start_x + j >= 2 && start_x + j <= 3 && start_y + i >= 2 && start_y + i <= 3) {
                map(start_y + i, start_x + j) = 1;
            }
        }
    }
    
    // Add obstacles around goal position (6.0, 6.0)
    std::size_t goal_x = static_cast<std::size_t>((6.0 - kMapXMin) / kMapResolution);
    std::size_t goal_y = static_cast<std::size_t>((6.0 - kMapYMin) / kMapResolution);
    
    // Add a wall-like obstacle near goal
    for (std::size_t i = 0; i < 20 && goal_y + i < map_row; ++i) {
        if (goal_x + 5 < map_col) {
            map(goal_y + i, goal_x + 5) = 1;
        }
    }
    
    // Add another horizontal obstacle
    for (std::size_t j = 0; j < 10 && goal_x + j < map_col; ++j) {
        if (goal_y + 3 < map_row) {
            map(goal_y + 3, goal_x + j) = 1;
        }
    }
    
    return map;
}

int main()
{
    HybridAStarPlanner planner;
    planner.setConfigParameters(kNumDirectionDivision, kStepLength);
    planner.setCostWeights(kCostWeightReverse, kCostWeightDirectionChange, kCostWeightSteeringAngle, kCostWeightSteeringRate, kCostWeightHybrid);
    planner.setMapParameters(kMapResolution, kMapXMin, kMapYMin, kMapXMax, kMapYMax, std::move(generateMap1()));
    planner.setVehicleParameters(kVehicleMaxSteeringAngle, kVehicleSteerPrecision, kVehicleWheelbase, kVehicleAxleToFront, kVehicleAxleToRear, kVehicleWidth, kVehicleEnableReverse);
    auto path = planner.plan({{0.0, 0.0, 0.0}}, {{6.0, 6.0, M_PI_2}});
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