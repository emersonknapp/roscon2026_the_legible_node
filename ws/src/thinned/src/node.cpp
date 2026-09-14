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

#include "thin_ogmof/node.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_map_outlier_filter::Node)

namespace occupancy_grid_map_outlier_filter
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("OccupancyGridMapOutlierFilter", options)
{
  params_.map_frame = declare_parameter<std::string>("map_frame");
  params_.base_link_frame = declare_parameter<std::string>("base_link_frame");
  params_.cost_threshold = declare_parameter<int>("cost_threshold");
  params_.use_radius_search_2d_filter = declare_parameter<bool>("use_radius_search_2d_filter");
  params_.enable_debugger = declare_parameter<bool>("enable_debugger");

  // if (use_radius_search_2d_filter) {
  //   RadiusSearch2dParams radius{};
  //   radius.search_radius = declare_parameter<float>("radius_search_2d_filter.search_radius");
  //   radius.min_points_and_distance_ratio =
  //     declare_parameter<float>("radius_search_2d_filter.min_points_and_distance_ratio");
  //   radius.min_points = declare_parameter<int>("radius_search_2d_filter.min_points");
  //   radius.max_points = declare_parameter<int>("radius_search_2d_filter.max_points");
  //   radius.max_filter_points_nb =
  //     static_cast<uint32_t>(declare_parameter<int>("radius_search_2d_filter.max_filter_points_nb"));
  //   params_.radius_search = radius;
  // }

  // tf2_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);

  // pointcloud_sub_.subscribe(this, "~/input/pointcloud", rclcpp::SensorDataQoS());
  // occupancy_grid_map_sub_.subscribe(this, "~/input/occupancy_grid_map", rclcpp::QoS{1});
  // sync_ptr_ = std::make_shared<Sync>(SyncPolicy(5), occupancy_grid_map_sub_, pointcloud_sub_);
  // sync_ptr_->registerCallback(std::bind(
  //   &Node::onOccupancyGridMapAndPointCloud2,
  //   this,
  //   std::placeholders::_1,
  //   std::placeholders::_2));

  // pointcloud_pub_ = create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::QoS{5}.reliable());

  // if (enable_debugger_) {
  //   high_confidence_pub_ =
  //     create_publisher<PointCloud2>("~/output/debug/high_confidence/pointcloud", rclcpp::SensorDataQoS());
  //   low_confidence_pub_ =
  //     create_publisher<PointCloud2>("~/output/debug/low_confidence/pointcloud", rclcpp::SensorDataQoS());
  //   outlier_pub_ = create_publisher<PointCloud2>("~/output/debug/outlier/pointcloud", rclcpp::SensorDataQoS());
  // }
}

}  // namespace occupancy_grid_map_outlier_filter
