#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore
{
public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void markObstacle(double x, double y);
    void clearGrid();
    void inflateObstacles();
    nav_msgs::msg::OccupancyGrid getGridMessage() const;

    void markFreeRay(double x, double y);

private:
    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 400;
    int height_ = 400;

    double origin_x_ = -20.0;
    double origin_y_ = -20.0;
    double inflation_radius_ = 1.0;  // metres

    std::vector<std::vector<std::int8_t>> grid_;
};

}

#endif