#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger)
{
    grid_.assign(
        height_,
        std::vector<std::int8_t>(width_, -1));
}

void CostmapCore::markObstacle(double x, double y)
{
    const double column =
        std::floor((x - origin_x_) / resolution_);

    const double row =
        std::floor((y - origin_y_) / resolution_);

    if (column < 0 || column >= width_ ||
        row < 0 || row >= height_) {
        return;
    }

    grid_[static_cast<std::size_t>(row)]
         [static_cast<std::size_t>(column)] = 100;
}

void CostmapCore::clearGrid()
{
    for (auto& row : grid_) {
        std::fill(row.begin(), row.end(), -1);
    }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getGridMessage() const
{
    nav_msgs::msg::OccupancyGrid message;

    message.info.resolution = resolution_;
    message.info.width = width_;
    message.info.height = height_;

    message.info.origin.position.x = origin_x_;
    message.info.origin.position.y = origin_y_;
    message.info.origin.position.z = 0.0;
    message.info.origin.orientation.w = 1.0;

    message.data.reserve(width_ * height_);

    for (const auto& row : grid_) {
        message.data.insert(
            message.data.end(), row.begin(), row.end());
    }

    return message;
}

void CostmapCore::inflateObstacles()
{
    if (inflation_radius_ <= 0.0) {
        return;
    }

    // Keep a snapshot of the original obstacle cells.
    const auto original_grid = grid_;

    const int radius_cells = static_cast<int>(
        std::ceil(inflation_radius_ / resolution_));

    for (int row = 0; row < height_; ++row) {
        for (int column = 0; column < width_; ++column) {
            if (original_grid[row][column] != 100) {
                continue;
            }

            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    const int neighbour_row = row + dy;
                    const int neighbour_column = column + dx;

                    if (neighbour_row < 0 || neighbour_row >= height_ ||
                        neighbour_column < 0 || neighbour_column >= width_) {
                        continue;
                    }

                    const double distance =
                        std::hypot(dx, dy) * resolution_;

                    if (distance > inflation_radius_) {
                        continue;
                    }

                    const auto cost = static_cast<std::int8_t>(
                        std::lround(
                            100.0 * (1.0 - distance / inflation_radius_)));

                    auto& cell = grid_[neighbour_row][neighbour_column];
                    cell = std::max(cell, cost);
                }
            }
        }
    }
}

void CostmapCore::markFreeRay(double x, double y)
{
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return;
    }

    const double distance = std::hypot(x, y);

    if (distance <= 0.0) {
        return;
    }

    // Sample every half-cell along the beam.
    const double step = resolution_ * 0.5;

    const double endpoint_column =
        std::floor((x - origin_x_) / resolution_);

    const double endpoint_row =
        std::floor((y - origin_y_) / resolution_);

    for (double travelled = 0.0;
         travelled < distance;
         travelled += step) {
        const double fraction = travelled / distance;

        const double ray_x = fraction * x;
        const double ray_y = fraction * y;

        const double column =
            std::floor((ray_x - origin_x_) / resolution_);

        const double row =
            std::floor((ray_y - origin_y_) / resolution_);

        if (column < 0 || column >= width_ ||
            row < 0 || row >= height_) {
            continue;
        }

        // Leave the endpoint cell for obstacle marking.
        if (column == endpoint_column && row == endpoint_row) {
            continue;
        }

        auto& cell =
            grid_[static_cast<std::size_t>(row)]
                 [static_cast<std::size_t>(column)];

        // Never erase an obstacle found by another beam.
        if (cell == -1) {
            cell = 0;
        }
    }
}


}