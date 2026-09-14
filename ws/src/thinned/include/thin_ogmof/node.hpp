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

#include <Eigen/Core>
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include <string>

namespace occupancy_grid_map_outlier_filter
{

class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options);

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  // ROS parameters
  struct Parameters
  {
    std::string map_frame = "map";
    std::string base_link_frame = "base_link";
    int cost_threshold = 45;
    bool use_radius_search_2d_filter = true;
    bool enable_debugger = false;
  };
  Parameters params_;

  // ROS communications interface
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;

  message_filters::Subscriber<OccupancyGrid> occupancy_grid_map_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  using SyncPolicy = message_filters::sync_policies::ExactTime<OccupancyGrid, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_ptr_;


  // Implementation libraries
  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

};

}  // namespace occupancy_grid_map_outlier_filter
