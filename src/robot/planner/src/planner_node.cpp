#include "planner_node.hpp"

#include <cmath>
#include <functional>
#include <memory>

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(this->get_logger())
{
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path", 10);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
            map_ = message;
            map_received_ = std::chrono::steady_clock::now();
        });

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered",
        rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
            odometry_ = message;
            odometry_received_ = std::chrono::steady_clock::now();
        });

    goal_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr message) {
                if (message->header.frame_id != "sim_world" ||
                    !std::isfinite(message->point.x) ||
                    !std::isfinite(message->point.y)) {
                    goal_active_ = false;
                    publishEmptyPath();
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal rejected: use finite coordinates in sim_world.");
                    return;
                }

                goal_ = *message;
                goal_active_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "New goal: x=%.2f, y=%.2f",
                    goal_.point.x, goal_.point.y);

                replan();
            });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::replan, this));
}

void PlannerNode::publishEmptyPath()
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";

    if (odometry_) {
        path.header.stamp = odometry_->header.stamp;
    }

    path_pub_->publish(path);
}

void PlannerNode::replan()
{
    if (!goal_active_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!map_ || !odometry_ ||
        now - map_received_ > std::chrono::seconds(2) ||
        now - odometry_received_ > std::chrono::seconds(2)) {
        publishEmptyPath();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Waiting for fresh map and odometry.");
        return;
    }

    if (map_->header.frame_id != "sim_world" ||
        odometry_->header.frame_id != "sim_world" ||
        odometry_->child_frame_id != "robot/chassis/lidar") {
        publishEmptyPath();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Map or odometry frame mismatch.");
        return;
    }

    const auto& position = odometry_->pose.pose.position;

    if (std::hypot(
            goal_.point.x - position.x,
            goal_.point.y - position.y) < 0.3) {
        goal_active_ = false;
        publishEmptyPath();
        RCLCPP_INFO(this->get_logger(), "Goal reached.");
        return;
    }

    auto path = planner_.plan(
        *map_,
        position.x, position.y,
        goal_.point.x, goal_.point.y);

    path.header.stamp = odometry_->header.stamp;
    for (auto& pose : path.poses) {
        pose.header = path.header;
    }

    if (path.poses.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "No path: start/goal blocked, unknown, outside map, or disconnected.");
    } else {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Path contains %zu poses.", path.poses.size());
    }

    // An empty path also invalidates any previously published route.
    path_pub_->publish(path);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}