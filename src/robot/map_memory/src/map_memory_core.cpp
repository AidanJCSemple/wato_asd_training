#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger)
{
    grid_.assign(width_ * height_, -1);
}

void MapMemoryCore::updateMap(
    const nav_msgs::msg::OccupancyGrid& local_map,
    double sensor_x,
    double sensor_y,
    double sensor_yaw)
{
    const auto& info = local_map.info;

    const std::size_t expected_size =
        static_cast<std::size_t>(info.width) * info.height;

    if (info.resolution <= 0.0 ||
        local_map.data.size() != expected_size ||
        !std::isfinite(sensor_x) ||
        !std::isfinite(sensor_y) ||
        !std::isfinite(sensor_yaw)) {
        return;
    }

    const double cosine = std::cos(sensor_yaw);
    const double sine = std::sin(sensor_yaw);
    // Collect this scan's observations before updating memory.
    std::vector<std::int8_t> observed(grid_.size(), -1);

    for (std::uint32_t row = 0; row < info.height; ++row) {
        for (std::uint32_t column = 0; column < info.width; ++column) {
            const std::size_t local_index =
                static_cast<std::size_t>(row) * info.width + column;

            const auto cost = local_map.data[local_index];

            // This first version remembers only positive costs.
            if (cost < 0) {
              continue;
            }

            // Centre of this cell in the LiDAR frame.
            // Our local grid's origin orientation is identity.
            const double local_x =
                info.origin.position.x +
                (column + 0.5) * info.resolution;

            const double local_y =
                info.origin.position.y +
                (row + 0.5) * info.resolution;

            // Rotate into world orientation, then translate.
            const double world_x =
                sensor_x + local_x * cosine - local_y * sine;

            const double world_y =
                sensor_y + local_x * sine + local_y * cosine;

            // Convert world position into a global grid cell.
            const double global_column =
                std::floor((world_x - origin_x_) / resolution_);

            const double global_row =
                std::floor((world_y - origin_y_) / resolution_);

            if (global_column < 0 || global_column >= width_ ||
                global_row < 0 || global_row >= height_) {
                continue;
            }

            const std::size_t global_index =
                static_cast<std::size_t>(global_row) * width_ +
                static_cast<std::size_t>(global_column);

           observed[global_index] =
            std::max(observed[global_index], cost);
        }
    }
    // Update only locations observed in this scan.
  for (std::size_t i = 0; i < grid_.size(); ++i) {
    if (observed[i] >= 0) {
        grid_[i] = observed[i];
      }
    }

}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMapMessage() const
{
    nav_msgs::msg::OccupancyGrid message;

    message.info.resolution = resolution_;
    message.info.width = width_;
    message.info.height = height_;

    message.info.origin.position.x = origin_x_;
    message.info.origin.position.y = origin_y_;
    message.info.origin.position.z = 0.0;
    message.info.origin.orientation.w = 1.0;

    message.data = grid_;

    return message;
}

}