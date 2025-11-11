#include <data_saver.h>

bool DataSaver::saveMapAndPath(const std::string &filename, const MapAndPathData &data)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    // Write map parameters
    file.write(reinterpret_cast<const char *>(&data.resolution), sizeof(data.resolution));
    file.write(reinterpret_cast<const char *>(&data.x_min), sizeof(data.x_min));
    file.write(reinterpret_cast<const char *>(&data.x_max), sizeof(data.x_max));
    file.write(reinterpret_cast<const char *>(&data.y_min), sizeof(data.y_min));
    file.write(reinterpret_cast<const char *>(&data.y_max), sizeof(data.y_max));
    file.write(reinterpret_cast<const char *>(&data.rows), sizeof(data.rows));
    file.write(reinterpret_cast<const char *>(&data.cols), sizeof(data.cols));

    // Write map data size and content
    std::size_t map_size = data.map_data.size();
    file.write(reinterpret_cast<const char *>(&map_size), sizeof(map_size));
    if (!data.map_data.empty())
    {
        file.write(reinterpret_cast<const char *>(data.map_data.data()), map_size * sizeof(uint8_t));
    }

    // Write path size
    std::size_t path_size = data.path.size();
    file.write(reinterpret_cast<const char *>(&path_size), sizeof(path_size));

    // Write path data
    for (const auto &waypoint : data.path)
    {
        file.write(reinterpret_cast<const char *>(waypoint.data()), 3 * sizeof(float));
    }

    // Write vehicle parameters
    file.write(reinterpret_cast<const char *>(&data.vehicle_wheelbase), sizeof(data.vehicle_wheelbase));
    file.write(reinterpret_cast<const char *>(&data.vehicle_axle_to_front), sizeof(data.vehicle_axle_to_front));
    file.write(reinterpret_cast<const char *>(&data.vehicle_axle_to_rear), sizeof(data.vehicle_axle_to_rear));
    file.write(reinterpret_cast<const char *>(&data.vehicle_width), sizeof(data.vehicle_width));

    file.close();
    return true;
}

MapAndPathData DataSaver::loadMapAndPath(const std::string &filename)
{
    MapAndPathData data = {};
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        return data;
    }

    // Read map parameters
    file.read(reinterpret_cast<char *>(&data.resolution), sizeof(data.resolution));
    file.read(reinterpret_cast<char *>(&data.x_min), sizeof(data.x_min));
    file.read(reinterpret_cast<char *>(&data.x_max), sizeof(data.x_max));
    file.read(reinterpret_cast<char *>(&data.y_min), sizeof(data.y_min));
    file.read(reinterpret_cast<char *>(&data.y_max), sizeof(data.y_max));
    file.read(reinterpret_cast<char *>(&data.rows), sizeof(data.rows));
    file.read(reinterpret_cast<char *>(&data.cols), sizeof(data.cols));

    // Read map data size and content
    std::size_t map_size;
    file.read(reinterpret_cast<char *>(&map_size), sizeof(map_size));
    data.map_data.resize(map_size);
    if (map_size > 0)
    {
        file.read(reinterpret_cast<char *>(data.map_data.data()), map_size * sizeof(uint8_t));
    }

    // Read path size
    std::size_t path_size;
    file.read(reinterpret_cast<char *>(&path_size), sizeof(path_size));

    // Read path data
    data.path.resize(path_size);
    for (std::size_t i = 0; i < path_size; ++i)
    {
        file.read(reinterpret_cast<char *>(data.path[i].data()), 3 * sizeof(float));
    }

    // Read vehicle parameters
    file.read(reinterpret_cast<char *>(&data.vehicle_wheelbase), sizeof(data.vehicle_wheelbase));
    file.read(reinterpret_cast<char *>(&data.vehicle_axle_to_front), sizeof(data.vehicle_axle_to_front));
    file.read(reinterpret_cast<char *>(&data.vehicle_axle_to_rear), sizeof(data.vehicle_axle_to_rear));
    file.read(reinterpret_cast<char *>(&data.vehicle_width), sizeof(data.vehicle_width));

    file.close();
    return data;
}