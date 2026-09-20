#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode();

private:
    void replan();
    void publishEmptyPath();

    robot::PlannerCore planner_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;
    geometry_msgs::msg::PointStamped goal_;
    bool goal_active_ = false;

    std::chrono::steady_clock::time_point map_received_;
    std::chrono::steady_clock::time_point odometry_received_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif