// Copyright 2021 Tier IV, Inc. All rights reserved.
// Copyright 2026 Alistair English, Emerson Knapp
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

// Thin ROS shell around the functional core in src/outlier_filter. This class
// owns nothing but I/O: parameters, TF, the synchronized subscription and the
// publishers. All algorithmic work is delegated to
// filterOccupancyGridMapOutliers().

#pragma once

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <string>

#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "outlier_filter/outlier_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace thin_occupancy_grid_map_outlier_filter
{
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::PointCloud2;

class OccupancyGridMapOutlierFilterComponent : public rclcpp::Node
{
public:
  explicit OccupancyGridMapOutlierFilterComponent(const rclcpp::NodeOptions & options);

private:
  void onOccupancyGridMapAndPointCloud2(
    const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc);

  std::optional<Eigen::Matrix4f> lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr high_confidence_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr low_confidence_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr outlier_pub_;
  message_filters::Subscriber<OccupancyGrid> occupancy_grid_map_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  using SyncPolicy = message_filters::sync_policies::ExactTime<OccupancyGrid, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_ptr_;

  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::string base_link_frame_;
  OutlierFilterParams params_;
  bool enable_debugger_;
};
}  // namespace thin_occupancy_grid_map_outlier_filter
