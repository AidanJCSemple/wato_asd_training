#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
    MapMemoryNode();

private:
    robot::MapMemoryCore map_memory_;

    void costmapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr message);

    void odometryCallback(
        const nav_msgs::msg::Odometry::SharedPtr message);

    void publishMap();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        costmap_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
};

#endif