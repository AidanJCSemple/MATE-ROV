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

#include "ghost_brain/controller.hpp"
#include <algorithm>
#include <cmath>

namespace ghost_brain
{

geometry_msgs::msg::Twist Controller::compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path)
{
  geometry_msgs::msg::Twist command;

  if (path.empty()) {
    return command;
  }

  // Find the route point nearest our current position.
  size_t nearest = 0;
  double nearest_distance = std::hypot(
    path[0].x - pose.position.x,
    path[0].y - pose.position.y,
    path[0].z - pose.position.z);

  for (size_t i = 1; i < path.size(); ++i) {
    const double distance = std::hypot(
      path[i].x - pose.position.x,
      path[i].y - pose.position.y,
      path[i].z - pose.position.z);

    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }

  // Chase the next point rather than the point we are already at.
  const size_t target_index =
    nearest + 1 < path.size() ? nearest + 1 : nearest;

  const auto & target = path[target_index];

  const double dx = target.x - pose.position.x;
  const double dy = target.y - pose.position.y;
  const double dz = target.z - pose.position.z;

  const auto & q = pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Rotate the map-frame error into the ghost's frame.
  const double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
  const double left_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

  constexpr double position_gain = 1.0;

  command.linear.x = std::clamp(
    position_gain * forward_error, -1.0, 1.0);
  command.linear.y = std::clamp(
    position_gain * left_error, -0.6, 0.6);
  command.linear.z = std::clamp(
    position_gain * dz, -0.6, 0.6);

  if (std::hypot(dx, dy) > 0.05) {
    const double desired_yaw = std::atan2(dy, dx);
    const double heading_error = std::atan2(
      std::sin(desired_yaw - yaw),
      std::cos(desired_yaw - yaw));

    constexpr double heading_gain = 1.5;
    command.angular.z = std::clamp(
      heading_gain * heading_error, -1.5, 1.5);
  }

  return command;
}

}  // namespace ghost_brain
