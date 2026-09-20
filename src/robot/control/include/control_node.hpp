#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();

private:
    void controlLoop();

    robot::ControlCore control_;

    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;

    std::chrono::steady_clock::time_point path_received_;
    std::chrono::steady_clock::time_point odometry_received_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif