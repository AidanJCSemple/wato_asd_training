#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
    : logger_(logger)
{
}

geometry_msgs::msg::Twist ControlCore::calculateCommand(
    const nav_msgs::msg::Path& path,
    double x, double y, double yaw)
{
    geometry_msgs::msg::Twist command;

    if (path.poses.empty() ||
        !std::isfinite(x) ||
        !std::isfinite(y) ||
        !std::isfinite(yaw)) {
        return command;
    }

    for (const auto& pose : path.poses) {
        if (!std::isfinite(pose.pose.position.x) ||
            !std::isfinite(pose.pose.position.y)) {
            return command;
        }
    }

    const auto& goal = path.poses.back().pose.position;
    const double goal_distance = std::hypot(goal.x - x, goal.y - y);

    if (goal_distance <= goal_tolerance_) {
        return command;
    }

    // Find the closest waypoint on the current path.
    std::size_t closest = 0;
    double closest_distance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const auto& point = path.poses[i].pose.position;
        const double distance = std::hypot(point.x - x, point.y - y);

        if (distance < closest_distance) {
            closest_distance = distance;
            closest = i;
        }
    }

    // Stop if the robot is far away from the supplied route.
    if (closest_distance > 1.0) {
        return command;
    }

    // Walk forward along the path by the lookahead distance.
    auto target = path.poses[closest].pose.position;
    double remaining = lookahead_;

    for (std::size_t i = closest; i + 1 < path.poses.size(); ++i) {
        const auto& a = path.poses[i].pose.position;
        const auto& b = path.poses[i + 1].pose.position;
        const double length = std::hypot(b.x - a.x, b.y - a.y);

        if (length < 1e-9) {
            continue;
        }

        if (length >= remaining) {
            const double fraction = remaining / length;
            target.x = a.x + fraction * (b.x - a.x);
            target.y = a.y + fraction * (b.y - a.y);
            break;
        }

        remaining -= length;
        target = b;
    }

    const double dx = target.x - x;
    const double dy = target.y - y;

    // Express the target in the robot's heading coordinates.
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double distance_squared = dx * dx + dy * dy;

    if (distance_squared < 1e-8) {
        return command;
    }

    const double heading_error = std::atan2(local_y, local_x);

    // Turn first if the target is more than about 40 degrees away.
    if (std::abs(heading_error) > 0.7) {
        command.angular.z = std::clamp(
            1.5 * heading_error, -max_turn_rate_, max_turn_rate_);
        return command;
    }

    const double curvature = 2.0 * local_y / distance_squared;

    // Slow down near the goal and on tighter turns.
    double speed = std::min(max_speed_, 0.6 * goal_distance);
    speed /= 1.0 + std::abs(curvature);

    if (std::abs(curvature) > 1e-9) {
        speed = std::min(speed, max_turn_rate_ / std::abs(curvature));
    }

    command.linear.x = speed;
    command.angular.z = speed * curvature;

    return command;
}

}