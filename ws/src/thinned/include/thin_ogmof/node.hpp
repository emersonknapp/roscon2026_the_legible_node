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

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "Eigen/Core"
#include "message_filters/pass_through.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "thin_ogmof/outlier_filter.hpp"

namespace occupancy_grid_map_outlier_filter
{

class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options);

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  void sync_callback(const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc);

  // Configuration
  struct Parameters
  {
    std::string map_frame = "map";
    std::string base_link_frame = "base_link";
    int cost_threshold = 45;
    bool use_radius_search_2d_filter = true;
  };

  Parameters params_;

  // Communications interface
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_occupancy_grid_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;

  // Node interface extensions/mixins
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::optional<RadiusSearch2dFilterNodeMixin> radius_search_mixin_;

  // Implementation
  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::optional<RadiusSearch2dFilter> radius_search_2d_filter_;

  message_filters::PassThrough<OccupancyGrid> sync_pt_occupancy_grid_;
  message_filters::PassThrough<PointCloud2> sync_pt_pointcloud_;
  using SyncPolicy = message_filters::sync_policies::ExactTime<OccupancyGrid, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
};

}  // namespace occupancy_grid_map_outlier_filter
