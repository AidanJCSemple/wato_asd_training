#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(this->get_logger())
{
    costmap_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap",
            10,
            std::bind(
                &MapMemoryNode::costmapCallback,
                this,
                std::placeholders::_1));

    odometry_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered",
            rclcpp::SensorDataQoS(),
            std::bind(
                &MapMemoryNode::odometryCallback,
                this,
                std::placeholders::_1));

    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(1).reliable().transient_local());

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapMemoryNode::publishMap, this));
}

void MapMemoryNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
{
    latest_odometry_ = message;
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
    latest_costmap_ = message;

    if (!latest_odometry_) {
        return;
    }

    // Only combine messages whose coordinate frames match.
    if (message->header.frame_id != latest_odometry_->child_frame_id ||
        latest_odometry_->header.frame_id != "sim_world") {
        return;
    }

    // Both messages use simulation timestamps.
    const double scan_time =
        message->header.stamp.sec +
        message->header.stamp.nanosec * 1e-9;

    const double pose_time =
        latest_odometry_->header.stamp.sec +
        latest_odometry_->header.stamp.nanosec * 1e-9;

    // Avoid combining a scan with a substantially older pose.
    if (std::abs(scan_time - pose_time) > 0.2) {
        return;
    }

    const auto& pose = latest_odometry_->pose.pose;
    const auto& q = pose.orientation;

    // Extract planar heading (yaw) from the quaternion.
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    map_memory_.updateMap(
        *message,
        pose.position.x,
        pose.position.y,
        yaw);
}

void MapMemoryNode::publishMap()
{
    auto message = map_memory_.getMapMessage();
    message.header.frame_id = "sim_world";

    if (latest_odometry_) {
        message.header.stamp = latest_odometry_->header.stamp;
    }

    map_pub_->publish(message);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}