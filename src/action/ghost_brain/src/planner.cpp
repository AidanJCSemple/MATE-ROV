// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ghost_brain/planner.hpp"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <queue>
#include <vector>
#include <iostream>

namespace ghost_brain
{

bool is_occupied(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const double resolution = map.info.resolution;
  const auto col = static_cast<int64_t>(std::floor((x - map.info.origin.position.x) / resolution));
  const auto row = static_cast<int64_t>(std::floor((y - map.info.origin.position.y) / resolution));
  if (col < 0 || row < 0 || col >= map.info.width || row >= map.info.height) {
    return true;
  }
  return map.data[row * map.info.width + col] != 0;
}

Path Planner::plan(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  const double resolution = map.info.resolution;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      width <= 0 || height <= 0)
  {
    return {};
  }

  const size_t cell_count =
    static_cast<size_t>(width) * static_cast<size_t>(height);

  if (map.data.size() != cell_count) {
    return {};
  }

  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  const auto to_cell = [&](double x, double y, int & col, int & row) {
    const double grid_x = (x - origin_x) / resolution;
    const double grid_y = (y - origin_y) / resolution;

    if (!std::isfinite(grid_x) || !std::isfinite(grid_y) ||
        grid_x < 0.0 || grid_y < 0.0 ||
        grid_x >= width || grid_y >= height)
    {
      return false;
    }

    col = static_cast<int>(std::floor(grid_x));
    row = static_cast<int>(std::floor(grid_y));
    return true;
  };

  int start_col, start_row;
  int goal_col, goal_row;

  if (!to_cell(start.x, start.y, start_col, start_row) ||
      !to_cell(goal.x, goal.y, goal_col, goal_row))
  {
    return {};
  }

  constexpr double ghost_radius = 0.3;
  const int clearance_cells =
    static_cast<int>(std::ceil(ghost_radius / resolution)) + 1;

  std::vector<bool> blocked(cell_count, false);

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const size_t index = static_cast<size_t>(row) * width + col;

      const double center_x = (col + 0.5) * resolution;
      const double center_y = (row + 0.5) * resolution;

      // Keep the whole ghost within the map.
      if (center_x <= ghost_radius ||
          center_y <= ghost_radius ||
          width * resolution - center_x <= ghost_radius ||
          height * resolution - center_y <= ghost_radius)
      {
        blocked[index] = true;
        continue;
      }

      for (int dr = -clearance_cells;
           dr <= clearance_cells && !blocked[index]; ++dr)
      {
        for (int dc = -clearance_cells; dc <= clearance_cells; ++dc) {
          const int obstacle_col = col + dc;
          const int obstacle_row = row + dr;

          if (obstacle_col < 0 || obstacle_row < 0 ||
              obstacle_col >= width || obstacle_row >= height)
          {
            continue;
          }

          const size_t obstacle_index =
            static_cast<size_t>(obstacle_row) * width + obstacle_col;

          if (map.data[obstacle_index] == 0) {
            continue;
          }

          // Distance from this centre to the nearest edge of the obstacle cell.
          const double dx = std::max(
            std::abs(dc) * resolution - resolution / 2.0, 0.0);
          const double dy = std::max(
            std::abs(dr) * resolution - resolution / 2.0, 0.0);

          if (dx * dx + dy * dy <= ghost_radius * ghost_radius) {
            blocked[index] = true;
            break;
          }
        }
      }
    }
  }

  const size_t start_index =
    static_cast<size_t>(start_row) * width + start_col;
  const size_t goal_index =
    static_cast<size_t>(goal_row) * width + goal_col;

  if (blocked[start_index] || blocked[goal_index]) {
    std::cerr << "Planner rejected endpoint: start_blocked="
              << blocked[start_index]
              << ", goal_blocked=" << blocked[goal_index]
              << '\n';
    return {};
  }

  // cell_count is an invalid index, so it means "not visited".
  std::vector<size_t> parent(cell_count, cell_count);
  std::queue<size_t> frontier;

  parent[start_index] = start_index;
  frontier.push(start_index);

  const int col_steps[4] = {1, -1, 0, 0};
  const int row_steps[4] = {0, 0, 1, -1};

  while (!frontier.empty()) {
    const size_t current = frontier.front();
    frontier.pop();

    if (current == goal_index) {
      break;
    }

    const int col = static_cast<int>(current % width);
    const int row = static_cast<int>(current / width);

    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + col_steps[direction];
      const int next_row = row + row_steps[direction];

      if (next_col < 0 || next_row < 0 ||
          next_col >= width || next_row >= height)
      {
        continue;
      }

      const size_t next =
        static_cast<size_t>(next_row) * width + next_col;

      if (blocked[next] || parent[next] != cell_count) {
        continue;
      }

      parent[next] = current;
      frontier.push(next);
    }
  }

    if (parent[goal_index] == cell_count) {
    std::cerr << "Planner: safe endpoints, but no connecting route\n";
    return {};
  }

  // Follow the parent links backward from goal to start.
  std::vector<size_t> route;

  for (size_t current = goal_index;; current = parent[current]) {
    route.push_back(current);

    if (current == start_index) {
      break;
    }
  }

  std::reverse(route.begin(), route.end());

  // Convert grid-cell centres back into map coordinates.
  Path path;
  path.reserve(route.size());

  for (size_t i = 0; i < route.size(); ++i) {
    const int col = static_cast<int>(route[i] % width);
    const int row = static_cast<int>(route[i] / width);

    geometry_msgs::msg::Point waypoint;
    waypoint.x = origin_x + (col + 0.5) * resolution;
    waypoint.y = origin_y + (row + 0.5) * resolution;

    const double progress = route.size() > 1 ?
      static_cast<double>(i) / static_cast<double>(route.size() - 1) :
      1.0;

    waypoint.z = start.z + progress * (goal.z - start.z);

    path.push_back(waypoint);
  }

  return path;
}

}  // namespace ghost_brain
