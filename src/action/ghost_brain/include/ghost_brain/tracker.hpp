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

#pragma once

#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace ghost_brain
{

/// A person or piece of candy the ghost has seen, and where it thinks it is
struct TrackedTarget
{
  geometry_msgs::msg::Point position;
  bool is_person{false};
  rclcpp::Time last_seen;                       // the detection's own stamp
  rclcpp::Time seen_at{0, 0, RCL_ROS_TIME};     // the brain's clock when it was last seen
  int id{-1};                                   // unique per track
  int hits{1};
};

/// Where the camera sits on the ghost: this far forward, left and up from its center (m)
constexpr double kCameraForward = 0.25;
constexpr double kCameraLeft = 0.0;
constexpr double kCameraUp = 0.1;

/**
 * @brief The depth at a pixel of a depth image, in meters.
 *
 * Depth is measured along the camera's axis, not along the ray to the pixel. It is infinite where
 * nothing is in range, and NaN outside the image.
 */
float depth_at(const sensor_msgs::msg::Image & depth, int u, int v);

/**
 * @brief Keeps track of where the people and candy are.
 *
 * Detections are placed in the map with the depth image and the ghost's pose, matched to tracks
 * we already know about, and given a unique id. Tracks are removed when the node sees the score
 * go up, and person tracks expire when they have not been seen for a while.
 */
class Tracker
{
public:
  /// Forget everything. Called at the start of every game
  void reset();

  /// Used by the tests: the brain's clock is not known, so nothing is stamped with it
  void on_detections(
    const vision_msgs::msg::Detection2DArray & detections,
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Pose & ghost_pose);

  /// `now` is the brain's own clock, used to tell how long ago each target was seen
  void on_detections(
    const vision_msgs::msg::Detection2DArray & detections,
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Pose & ghost_pose,
    const rclcpp::Time & now);

  const std::vector<TrackedTarget> & targets() const {return targets_;}

  /// Remove the nearest track of this kind within max_distance. Returns its id, if any
  std::optional<int> remove_nearest(
    bool is_person, const geometry_msgs::msg::Point & near, double max_distance);

  void remove_id(int id);
  void remove_all(bool is_person);

  /// People move, so a sighting older than max_age_s no longer says where the person is
  void drop_stale_people(const rclcpp::Time & now, double max_age_s);

private:
  std::vector<TrackedTarget> targets_;
  int next_id_{0};
};

}  // namespace ghost_brain