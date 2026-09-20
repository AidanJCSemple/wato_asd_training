#include "control_node.hpp"

#include <cmath>
#include <functional>
#include <memory>

ControlNode::ControlNode()
    : Node("control"),
      control_(this->get_logger())
{
    this->declare_parameter<bool>("enabled", true);

    command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", rclcpp::QoS(1),
        [this](nav_msgs::msg::Path::SharedPtr message) {
            path_ = message;
            path_received_ = std::chrono::steady_clock::now();
        });

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
            odometry_ = message;
            odometry_received_ = std::chrono::steady_clock::now();
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop()
{
    // A default Twist contains zero speed and zero turn rate.
    geometry_msgs::msg::Twist command;

    if (!this->get_parameter("enabled").as_bool()) {
        command_pub_->publish(command);
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!path_ || !odometry_ ||
        path_->poses.empty() ||
        now - path_received_ > std::chrono::seconds(2) ||
        now - odometry_received_ > std::chrono::milliseconds(500)) {
        command_pub_->publish(command);
        return;
    }

    if (path_->header.frame_id != "sim_world" ||
        odometry_->header.frame_id != "sim_world" ||
        odometry_->child_frame_id != "robot/chassis/lidar") {
        command_pub_->publish(command);
        return;
    }

    const auto& pose = odometry_->pose.pose;
    const auto& q = pose.orientation;

    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    command = control_.calculateCommand(
        *path_,
        pose.position.x,
        pose.position.y,
        yaw);

    command_pub_->publish(command);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}