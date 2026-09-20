#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class PlannerCore
{
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path plan(
        const nav_msgs::msg::OccupancyGrid& map,
        double start_x, double start_y,
        double goal_x, double goal_y);

private:
    rclcpp::Logger logger_;
};

}

#endif