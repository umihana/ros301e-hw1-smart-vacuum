// map data for the floors, used by map_loader AND task_manager
#pragma once

# include <map>
# include <string>
# include <vector>
# include <cstdint>

namespace smart_vacuum
{

struct Room
{
    std::string name;
    double area;  // m^2
};

struct FloorMap
{
    int width;          // cells
    int height;         // cells
    double resolution;  // m per cell
    double init_x;      // start pose = docking station [m]
    double init_y;      // [m]
    double init_theta;  // [rad]
    std::vector<Room> rooms;
};

// the two floors from the HW (Floor_1, Floor_2), static so its only created once
inline const std::map<std::string, FloorMap> & floor_maps()
{
    static const std::map<std::string, FloorMap> maps = {
        {"Floor_1", {40, 30, 0.25, 0.5, 0.5, 0.0,
            {{"Living Room", 24.0}, {"Kitchen", 12.0}, {"Hallway", 6.0}}}},
        {"Floor_2", {32, 32, 0.25, 1.0, 7.0, 1.57,
            {{"Bedroom", 16.0}, {"Bathroom", 6.0}, {"Office", 10.0}}}},
    };
    return maps;
}

// fake grid map: 0 = free, 100 = wall (only the border is wall)
inline std::vector<int8_t> build_grid(int width, int height)
{
    std::vector<int8_t> grid(width * height, 0);
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            if (r == 0 || r == height - 1 || c == 0 || c == width - 1) {
                grid[r * width + c] = 100;
            }
        }
    }
    return grid;
}

}  // namespace smart_vacuum
