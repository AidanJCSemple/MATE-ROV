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

#include "ghost_brain/tracker.hpp"

#include <cstring>
#include <limits>
#include <cmath>

namespace ghost_brain
{

float depth_at(const sensor_msgs::msg::Image & depth, int u, int v)
{
  if (u < 0 || v < 0 || u >= static_cast<int>(depth.width) || v >= static_cast<int>(depth.height)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float value = 0.0f;
  std::memcpy(&value, &depth.data[v * depth.step + u * sizeof(float)], sizeof(float));
  return value;
}

void Tracker::reset()
{
  targets_.clear();
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  if (fx <= 0.0 || fy <= 0.0) {
    return;
  }

  for (const auto & detection : detections.detections) {
    if (detection.results.empty()) {
      continue;
    }

    const auto & label = detection.results[0].hypothesis.class_id;
    if (label != "person" && label != "candy") {
      continue;
    }

    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;

    const double camera_z = depth_at(
      depth,
      static_cast<int>(std::round(u)),
      static_cast<int>(std::round(v)));

    if (!std::isfinite(camera_z) || camera_z <= 0.0) {
      continue;
    }

    const double camera_x = (u - cx) * camera_z / fx;
    const double camera_y = (v - cy) * camera_z / fy;

    const double ghost_x = camera_z + kCameraForward;
    const double ghost_y = -camera_x + kCameraLeft;
    const double ghost_z = -camera_y + kCameraUp;

    const auto & q = ghost_pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    TrackedTarget target;
    target.position.x =
      ghost_pose.position.x +
      ghost_x * std::cos(yaw) - ghost_y * std::sin(yaw);

    target.position.y =
      ghost_pose.position.y +
      ghost_x * std::sin(yaw) + ghost_y * std::cos(yaw);

    target.position.z = ghost_pose.position.z + ghost_z;

    target.is_person = (label == "person");
    target.last_seen = rclcpp::Time(detections.header.stamp);

    constexpr double match_distance = 0.6;

    TrackedTarget * closest = nullptr;
    double closest_distance_squared = match_distance * match_distance;

    for (auto & existing : targets_) {
      if (existing.is_person != target.is_person) {
        continue;
      }

      const double dx = existing.position.x - target.position.x;
      const double dy = existing.position.y - target.position.y;
      const double dz = existing.position.z - target.position.z;
      const double distance_squared = dx * dx + dy * dy + dz * dz;

      if (distance_squared < closest_distance_squared) {
        closest = &existing;
        closest_distance_squared = distance_squared;
      }
    }

    if (closest != nullptr) {
      closest->position = target.position;
      closest->last_seen = target.last_seen;
    } else {
      target.id = next_id_++;  // NEW: give the new track its own ID
      targets_.push_back(target);
    }
  }
}

}  // namespace ghost_brain