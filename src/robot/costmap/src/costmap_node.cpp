#include <chrono>
#include <functional>
#include <memory>
#include <cmath>


#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_(robot::CostmapCore(this->get_logger()))
{
    string_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/test_topic", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CostmapNode::publishMessage, this));
    
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar",
    rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::publishMessage()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!";

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing: '%s'",
        message.data.c_str());

    string_pub_->publish(message);
}

void CostmapNode::lidarCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    costmap_.clearGrid();
    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
        const double range = scan->ranges[i];
        const double angle =
            scan->angle_min + i * scan->angle_increment;

        if (!std::isfinite(range) ||
            range < scan->range_min ||
            range > scan->range_max) {
            continue;
        }

        const double x = range * std::cos(angle);
        const double y = range * std::sin(angle);

        costmap_.markFreeRay(x, y);
        costmap_.markObstacle(x, y);

        if (i % 64 == 0) {
            RCLCPP_INFO(
                this->get_logger(),
                "LiDAR[%zu]: angle=%.2f rad, x=%.2f m, y=%.2f m",
                i, angle, x, y);
        }
    }
    costmap_.inflateObstacles();

    auto message = costmap_.getGridMessage();
    message.header = scan->header;
    costmap_pub_->publish(message);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}