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
bool sphere_hits_box(
  const geometry_msgs::msg::Point & center,
  double radius,
  const haunted_house_msgs::msg::Box & box)
{
  const double nearest_x = std::clamp(center.x, box.min.x, box.max.x);
  const double nearest_y = std::clamp(center.y, box.min.y, box.max.y);
  const double nearest_z = std::clamp(center.z, box.min.z, box.max.z);

  const double dx = center.x - nearest_x;
  const double dy = center.y - nearest_y;
  const double dz = center.z - nearest_z;

  return dx * dx + dy * dy + dz * dz <= radius * radius;
}

bool segment_hits_box(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double radius,
  const haunted_house_msgs::msg::Box & box)
{
  double enter = 0.0;
  double leave = 1.0;

  const double starts[3] = {start.x, start.y, start.z};
  const double ends[3] = {end.x, end.y, end.z};

  const double mins[3] = {
    box.min.x - radius,
    box.min.y - radius,
    box.min.z - radius};

  const double maxs[3] = {
    box.max.x + radius,
    box.max.y + radius,
    box.max.z + radius};

  for (int axis = 0; axis < 3; ++axis) {
    const double direction = ends[axis] - starts[axis];

    if (std::abs(direction) < 1e-12) {
      if (starts[axis] < mins[axis] || starts[axis] > maxs[axis]) {
        return false;
      }
      continue;
    }

    double first = (mins[axis] - starts[axis]) / direction;
    double second = (maxs[axis] - starts[axis]) / direction;

    if (first > second) {
      std::swap(first, second);
    }

    enter = std::max(enter, first);
    leave = std::min(leave, second);

    if (enter > leave) {
      return false;
    }
  }

  return true;
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

Path Planner::plan_3d(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map,
  const haunted_house_msgs::msg::BoxArray & obstacles) const
{
  constexpr double radius = 0.3;
  constexpr double margin = 0.01;

  // The documented house has its floor at 0 m and ceiling at 3 m.
  constexpr double min_z = radius + margin;
  constexpr double max_z = 3.0 - radius - margin;

  if (!std::isfinite(map.info.resolution) ||
      map.info.resolution <= 0.0 ||
      map.info.width == 0 || map.info.height == 0)
  {
    return {};
  }

  // A coarser search grid keeps the 3D search manageable.
  const double step = std::max(
    0.2, static_cast<double>(map.info.resolution));

  const double min_x = map.info.origin.position.x + radius + margin;
  const double min_y = map.info.origin.position.y + radius + margin;

  const double max_x = map.info.origin.position.x +
    map.info.width * map.info.resolution - radius - margin;
  const double max_y = map.info.origin.position.y +
    map.info.height * map.info.resolution - radius - margin;

  const auto position_clear = [&](const geometry_msgs::msg::Point & p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z) ||
        p.x < min_x || p.x > max_x ||
        p.y < min_y || p.y > max_y ||
        p.z < min_z || p.z > max_z)
    {
      return false;
    }

    for (const auto & box : obstacles.boxes) {
      if (sphere_hits_box(p, radius + margin, box)) {
        return false;
      }
    }

    return true;
  };

  const auto movement_clear = [&](
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b)
  {
    if (!position_clear(a) || !position_clear(b)) {
      return false;
    }

    for (const auto & box : obstacles.boxes) {
      if (segment_hits_box(a, b, radius + margin, box)) {
        return false;
      }
    }

    return true;
  };

  // Next: create the grid and search through clear movements.
  if (max_x < min_x || max_y < min_y || !position_clear(start) ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      !std::isfinite(goal.z))
  {
    return {};
  }

  // Stop inside the 0.5 m collection radius, leaving some error allowance.
  constexpr double approach_distance = 0.4;

  const auto near_goal = [&](const geometry_msgs::msg::Point & p) {
    return std::hypot(
      p.x - goal.x, p.y - goal.y, p.z - goal.z) <= approach_distance;
  };

  if (near_goal(start)) {
    return Path{start};
  }

  const int nx = static_cast<int>(std::floor((max_x - min_x) / step)) + 1;
  const int ny = static_cast<int>(std::floor((max_y - min_y) / step)) + 1;
  const int nz = static_cast<int>(std::floor((max_z - min_z) / step)) + 1;

  const size_t total =
    static_cast<size_t>(nx) * static_cast<size_t>(ny) *
    static_cast<size_t>(nz);

  const auto index_of = [&](int x, int y, int z) -> size_t {
    return (static_cast<size_t>(z) * ny + y) * nx + x;
  };

  const auto point_of = [&](size_t index) {
    const int x = static_cast<int>(index % nx);
    const size_t remainder = index / nx;
    const int y = static_cast<int>(remainder % ny);
    const int z = static_cast<int>(remainder / ny);

    geometry_msgs::msg::Point p;
    p.x = min_x + x * step;
    p.y = min_y + y * step;
    p.z = min_z + z * step;
    return p;
  };

  // "total" is outside the valid index range and means unvisited.
  std::vector<size_t> parent(total, total);
  std::queue<size_t> frontier;

  const int sx = std::clamp(
    static_cast<int>(std::round((start.x - min_x) / step)), 0, nx - 1);
  const int sy = std::clamp(
    static_cast<int>(std::round((start.y - min_y) / step)), 0, ny - 1);
  const int sz = std::clamp(
    static_cast<int>(std::round((start.z - min_z) / step)), 0, nz - 1);

  // Connect the exact starting position to nearby clear grid points.
  // Using several candidates avoids relying on one rounded cell.
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = sx + dx;
        const int y = sy + dy;
        const int z = sz + dz;

        if (x < 0 || x >= nx || y < 0 || y >= ny ||
            z < 0 || z >= nz)
        {
          continue;
        }

        const size_t index = index_of(x, y, z);

        if (movement_clear(start, point_of(index))) {
          parent[index] = index;
          frontier.push(index);
        }
      }
    }
  }

  const int x_steps[6] = {1, -1, 0, 0, 0, 0};
  const int y_steps[6] = {0, 0, 1, -1, 0, 0};
  const int z_steps[6] = {0, 0, 0, 0, 1, -1};

  size_t reached = total;

  while (!frontier.empty()) {
    const size_t current = frontier.front();
    frontier.pop();

    const auto current_point = point_of(current);

    if (near_goal(current_point)) {
      reached = current;
      break;
    }

    const int x = static_cast<int>(current % nx);
    const size_t remainder = current / nx;
    const int y = static_cast<int>(remainder % ny);
    const int z = static_cast<int>(remainder / ny);

    for (int direction = 0; direction < 6; ++direction) {
      const int next_x = x + x_steps[direction];
      const int next_y = y + y_steps[direction];
      const int next_z = z + z_steps[direction];

      if (next_x < 0 || next_x >= nx ||
          next_y < 0 || next_y >= ny ||
          next_z < 0 || next_z >= nz)
      {
        continue;
      }

      const size_t next = index_of(next_x, next_y, next_z);

      if (parent[next] != total) {
        continue;
      }

      if (!movement_clear(current_point, point_of(next))) {
        continue;
      }

      parent[next] = current;
      frontier.push(next);
    }
  }

  if (reached == total) {
    return {};
  }

  Path path;

  for (size_t current = reached;; current = parent[current]) {
    path.push_back(point_of(current));

    if (parent[current] == current) {
      break;
    }
  }

  std::reverse(path.begin(), path.end());
  path.insert(path.begin(), start);
  return path;
}
}  // namespace ghost_brain
