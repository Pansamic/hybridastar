#ifndef __DATA_SAVER_H__
#define __DATA_SAVER_H__

#include <fstream>
#include <vector>
#include <array>
#include <string>
#include <Eigen/Core>

enum class AlgorithmType {
    ASTAR = 0,
    HYBRID_ASTAR = 1
};

struct MapAndPathData
{
    // Algorithm type
    AlgorithmType algorithm_type;
    
    // Map parameters
    float resolution;
    float x_min, x_max;
    float y_min, y_max;
    std::size_t rows, cols;

    // Map data (stored as flat array from Eigen::Matrix)
    std::vector<uint8_t> map_data;

    // Path data
    std::vector<std::array<float, 3>> path; // x, y, theta for Hybrid A*, x, y, - for A*

    // Vehicle parameters (only used for Hybrid A*, but stored for consistency)
    float vehicle_wheelbase;
    float vehicle_axle_to_front;
    float vehicle_axle_to_rear;
    float vehicle_width;
};

class DataSaver
{
public:
    static bool saveMapAndPath(const std::string &filename, const MapAndPathData &data);
    static MapAndPathData loadMapAndPath(const std::string &filename);
};

#endif // __DATA_SAVER_H__