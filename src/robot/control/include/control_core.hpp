#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore
{
public:
    explicit ControlCore(const rclcpp::Logger& logger);

    geometry_msgs::msg::Twist calculateCommand(
        const nav_msgs::msg::Path& path,
        double x, double y, double yaw);

private:
    rclcpp::Logger logger_;

    double lookahead_ = 0.4;
    double max_speed_ = 0.25;
    double max_turn_rate_ = 0.6;
    double goal_tolerance_ = 0.3;
};

}

#endif