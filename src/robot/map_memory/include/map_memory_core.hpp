#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore
{
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void updateMap(
      const nav_msgs::msg::OccupancyGrid& local_map,
      double sensor_x,
      double sensor_y,
      double sensor_yaw);

    nav_msgs::msg::OccupancyGrid getMapMessage() const;
    

private:
    rclcpp::Logger logger_;

    double resolution_ = 0.2;
    int width_ = 400;
    int height_ = 400;

    double origin_x_ = -40.0;
    double origin_y_ = -40.0;

    std::vector<std::int8_t> grid_;
};

}

#endif