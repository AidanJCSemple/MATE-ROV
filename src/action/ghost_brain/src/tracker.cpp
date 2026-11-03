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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

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

namespace
{

// The depth image shows the front of a head: its center is about this much farther away
constexpr double kHeadDepthOffset = 0.2;

// The nearest valid depth in the middle half of a detection's box. NaN if there is none
float nearest_depth_in_box(
  const sensor_msgs::msg::Image & depth, double u, double v, double size_x, double size_y)
{
  const int center_u = static_cast<int>(std::round(u));
  const int center_v = static_cast<int>(std::round(v));
  const int half_u = std::max(0, static_cast<int>(std::floor(size_x / 4.0)));
  const int half_v = std::max(0, static_cast<int>(std::floor(size_y / 4.0)));

  float best = std::numeric_limits<float>::quiet_NaN();
  for (int row = center_v - half_v; row <= center_v + half_v; ++row) {
    for (int col = center_u - half_u; col <= center_u + half_u; ++col) {
      const float value = depth_at(depth, col, row);
      if (!std::isfinite(value) || value <= 0.0f) {
        continue;
      }
      if (!std::isfinite(best) || value < best) {
        best = value;
      }
    }
  }
  return best;
}

}  // namespace

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
  on_detections(detections, depth, camera_info, ghost_pose, rclcpp::Time(0, 0, RCL_ROS_TIME));
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose,
  const rclcpp::Time & now)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  if (fx <= 0.0 || fy <= 0.0) {
    return;
  }

  const auto & q = ghost_pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  for (const auto & detection : detections.detections) {
    if (detection.results.empty()) {
      continue;
    }

    const auto & label = detection.results[0].hypothesis.class_id;
    if (label != "person" && label != "candy") {
      continue;
    }
    const bool is_person = (label == "person");

    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;

    const double measured = nearest_depth_in_box(
      depth, u, v, detection.bbox.size_x, detection.bbox.size_y);

    if (!std::isfinite(measured) || measured <= 0.0) {
      continue;
    }

    const double camera_z = measured + (is_person ? kHeadDepthOffset : 0.0);
    const double camera_x = (u - cx) * camera_z / fx;
    const double camera_y = (v - cy) * camera_z / fy;

    // Optical frame (right, down, forward) to the ghost's frame (forward, left, up)
    const double ghost_x = camera_z + kCameraForward;
    const double ghost_y = -camera_x + kCameraLeft;
    const double ghost_z = -camera_y + kCameraUp;

    TrackedTarget target;
    target.position.x =
      ghost_pose.position.x + ghost_x * std::cos(yaw) - ghost_y * std::sin(yaw);
    target.position.y =
      ghost_pose.position.y + ghost_x * std::sin(yaw) + ghost_y * std::cos(yaw);
    target.position.z = ghost_pose.position.z + ghost_z;
    target.is_person = is_person;
    target.last_seen = rclcpp::Time(detections.header.stamp);
    target.seen_at = now;

    // People move, so they get a wider gate than candy
    const double match_distance = is_person ? 0.7 : 0.5;

    TrackedTarget * closest = nullptr;
    double closest_distance_squared = match_distance * match_distance;

    for (auto & existing : targets_) {
      if (existing.is_person != is_person) {
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
      if (is_person) {
        closest->position = target.position;
      } else {
        // Candy does not move: average out the noise, but let new sightings count
        closest->position.x = 0.5 * (closest->position.x + target.position.x);
        closest->position.y = 0.5 * (closest->position.y + target.position.y);
        closest->position.z = 0.5 * (closest->position.z + target.position.z);
      }
      closest->last_seen = target.last_seen;
      closest->seen_at = now;
      closest->hits += 1;
    } else {
      target.id = next_id_++;
      targets_.push_back(target);
    }
  }
}

std::optional<int> Tracker::remove_nearest(
  bool is_person, const geometry_msgs::msg::Point & near, double max_distance)
{
  int best = -1;
  double best_distance = max_distance;

  for (size_t i = 0; i < targets_.size(); ++i) {
    if (targets_[i].is_person != is_person) {
      continue;
    }
    const double d = std::hypot(
      targets_[i].position.x - near.x,
      targets_[i].position.y - near.y,
      targets_[i].position.z - near.z);
    if (d <= best_distance) {
      best_distance = d;
      best = static_cast<int>(i);
    }
  }

  if (best < 0) {
    return std::nullopt;
  }

  const int id = targets_[static_cast<size_t>(best)].id;
  targets_.erase(targets_.begin() + best);
  return id;
}

void Tracker::remove_id(int id)
{
  targets_.erase(
    std::remove_if(
      targets_.begin(), targets_.end(),
      [id](const TrackedTarget & t) {return t.id == id;}),
    targets_.end());
}

void Tracker::remove_all(bool is_person)
{
  targets_.erase(
    std::remove_if(
      targets_.begin(), targets_.end(),
      [is_person](const TrackedTarget & t) {return t.is_person == is_person;}),
    targets_.end());
}

void Tracker::drop_stale_people(const rclcpp::Time & now, double max_age_s)
{
  targets_.erase(
    std::remove_if(
      targets_.begin(), targets_.end(),
      [&](const TrackedTarget & t) {
        return t.is_person && (now - t.seen_at).seconds() > max_age_s;
      }),
    targets_.end());
}

}  // namespace ghost_brain