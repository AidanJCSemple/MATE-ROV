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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ghost_brain/controller.hpp"
#include "ghost_brain/localizer.hpp"
#include "ghost_brain/planner.hpp"
#include "ghost_brain/tracker.hpp"
#include "haunted_house_msgs/msg/box_array.hpp"
#include "haunted_house_msgs/msg/game_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ghost_brain
{

/**
 * @brief The ghost's brain: works out where it is and where everything is, then goes after it.
 *
 * Sensor messages go to Localizer (where am I?) and Tracker (where are the people and candy?). While
 * the game is running, a control loop at `control_rate_hz`:
 *  - removes a target when the score says it was collected
 *  - picks the target with the best points per travel distance, and stays committed to it
 *  - explores toward unseen parts of the house when there is nothing to chase
 *  - plans a 3D route with Planner (again every `replan_period_s`, because people move)
 *  - sends Controller's velocity on `cmd_vel`
 *
 * It publishes Localizer's pose as the map -> base_link transform, so Foxglove draws the sonar and
 * camera where the brain thinks the ghost is: when they line up with the house, localization works.
 *
 * Subscribes: `ghost/velocity`, `ghost/imu`, `sonar/scan`, `camera/depth`, `camera/camera_info`,
 * `camera/detections`, `map`, `haunted_house/obstacles`, `haunted_house/start_pose`,
 * `haunted_house/state`
 * Publishes: `cmd_vel`, `ghost_brain/pose`, `ghost_brain/path`, `ghost_brain/markers`, and TF
 */
class GhostBrainNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit GhostBrainNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Target selection
  double target_score(const TrackedTarget & target, const std::vector<float> & travel) const;
  std::optional<TrackedTarget> choose_target(const std::vector<float> & travel, double * score_out) const;
  void commit_target(const TrackedTarget & target, const geometry_msgs::msg::Point & here);
  void release_target();
  void on_collected(bool is_person);

  // Exploration
  void init_coverage();
  void update_coverage(const geometry_msgs::msg::Point & here);
  bool pick_exploration_goal(const std::vector<float> & travel, bool * any_unseen);

  void on_state(const haunted_house_msgs::msg::GameState & state);
  void on_detections(const vision_msgs::msg::Detection2DArray & detections);
  void update();
  void publish_command(const geometry_msgs::msg::Twist & twist);
  void publish_pose();
  void publish_path();
  void publish_markers(const std::optional<TrackedTarget> & target);
  void release_resources();

  // Parameters
  double control_rate_hz_{20.0};
  rclcpp::Duration replan_period_{rclcpp::Duration::from_seconds(0.5)};
  std::string map_frame_;
  std::string base_frame_;

  // What the ghost knows
  std::optional<geometry_msgs::msg::PoseStamped> start_pose_;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<haunted_house_msgs::msg::BoxArray> obstacles_;
  std::optional<haunted_house_msgs::msg::GameState> state_;
  std::optional<sensor_msgs::msg::Image> depth_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

  // What it is doing
  Localizer localizer_;
  Tracker tracker_;
  Planner planner_;
  Controller controller_;
  Path path_;
  std::optional<geometry_msgs::msg::Point> planned_goal_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  // The committed target
  std::optional<int> target_id_;
  std::unordered_map<int, rclcpp::Time> retry_after_;
  std::unordered_map<int, int> attempts_;
  std::optional<rclcpp::Time> arrival_time_;

  // Progress, for noticing when the ghost is stuck
  geometry_msgs::msg::Point progress_position_;
  rclcpp::Time progress_time_{0, 0, RCL_ROS_TIME};

  // The score at the last message, so a rise means something was collected
  bool have_counts_{false};
  uint32_t last_candies_eaten_{0};
  uint32_t last_people_haunted_{0};

  // Exploration: which parts of the house have been looked at
  std::vector<uint8_t> seen_;
  int seen_cols_{0};
  int seen_rows_{0};
  double seen_cell_{0.5};
  rclcpp::Time last_coverage_time_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::Point> explore_goal_;
  int explore_cell_{-1};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<haunted_house_msgs::msg::BoxArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<haunted_house_msgs::msg::GameState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ghost_brain