#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger)
{
}

nav_msgs::msg::Path PlannerCore::plan(
    const nav_msgs::msg::OccupancyGrid& map,
    double start_x, double start_y,
    double goal_x, double goal_y)
{
    nav_msgs::msg::Path path;
    path.header = map.header;

    // Validate dimensions before converting to signed indices.
    if (map.info.width == 0 || map.info.height == 0 ||
        map.info.width > 10000 || map.info.height > 10000 ||
        !std::isfinite(map.info.resolution) ||
        map.info.resolution <= 0.0) {
        return path;
    }

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const int count = width * height;
    const double resolution = map.info.resolution;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;

    if (map.data.size() != static_cast<std::size_t>(count) ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y)) {
        return path;
    }

    // Our map publisher creates an axis-aligned grid.
    const auto& orientation = map.info.origin.orientation;
    if (std::abs(orientation.x) > 1e-6 ||
        std::abs(orientation.y) > 1e-6 ||
        std::abs(orientation.z) > 1e-6 ||
        std::abs(std::abs(orientation.w) - 1.0) > 1e-6) {
        return path;
    }

    auto worldToIndex = [&](double x, double y) -> int {
        if (!std::isfinite(x) || !std::isfinite(y)) {
            return -1;
        }

        const double column = std::floor((x - origin_x) / resolution);
        const double row = std::floor((y - origin_y) / resolution);

        if (column < 0 || column >= width ||
            row < 0 || row >= height) {
            return -1;
        }

        return static_cast<int>(row) * width +
               static_cast<int>(column);
    };

    auto traversable = [&](int index) {
        return map.data[index] >= 0 && map.data[index] < 50;
    };

    const int start = worldToIndex(start_x, start_y);
    const int goal = worldToIndex(goal_x, goal_y);

    if (start < 0 || goal < 0 ||
        !traversable(start) || !traversable(goal)) {
        return path;
    }

    const int goal_column = goal % width;
    const int goal_row = goal / width;

    // Manhattan distance is appropriate for four-direction movement.
    auto heuristic = [&](int index) {
        return resolution *
            (std::abs(index % width - goal_column) +
             std::abs(index / width - goal_row));
    };

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distance(count, infinity);
    std::vector<int> parent(count, -1);
    std::vector<bool> closed(count, false);

    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>,
                        std::greater<Entry>> open;

    distance[start] = 0.0;
    open.emplace(heuristic(start), start);

    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};
    bool found = false;

    while (!open.empty()) {
        const int current = open.top().second;
        open.pop();

        if (closed[current]) {
            continue;
        }

        closed[current] = true;

        if (current == goal) {
            found = true;
            break;
        }

        const int column = current % width;
        const int row = current / width;

        for (int direction = 0; direction < 4; ++direction) {
            const int next_column = column + dx[direction];
            const int next_row = row + dy[direction];

            if (next_column < 0 || next_column >= width ||
                next_row < 0 || next_row >= height) {
                continue;
            }

            const int next = next_row * width + next_column;

            if (closed[next] || !traversable(next)) {
                continue;
            }

            // Penalize proximity to obstacles.
            const double step_cost =
                resolution * (1.0 + 4.0 * map.data[next] / 100.0);

            const double candidate = distance[current] + step_cost;

            if (candidate < distance[next]) {
                distance[next] = candidate;
                parent[next] = current;
                open.emplace(candidate + heuristic(next), next);
            }
        }
    }

    if (!found) {
        return path;
    }

    // Follow parent links backward, then reverse the result.
    std::vector<int> cells;
    for (int current = goal; current != -1; current = parent[current]) {
        cells.push_back(current);
    }
    std::reverse(cells.begin(), cells.end());

    for (const int index : cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x =
            origin_x + (index % width + 0.5) * resolution;
        pose.pose.position.y =
            origin_y + (index / width + 0.5) * resolution;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    // Preserve the requested endpoints within their checked cells.
    path.poses.front().pose.position.x = start_x;
    path.poses.front().pose.position.y = start_y;

    if (path.poses.size() == 1) {
        path.poses.push_back(path.poses.front());
    }

    path.poses.back().pose.position.x = goal_x;
    path.poses.back().pose.position.y = goal_y;

    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
        const auto& a = path.poses[i].pose.position;
        const auto& b = path.poses[i + 1].pose.position;
        const double yaw = std::atan2(b.y - a.y, b.x - a.x);

        path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
        path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    }

    path.poses.back().pose.orientation =
        path.poses[path.poses.size() - 2].pose.orientation;

    return path;
}

}