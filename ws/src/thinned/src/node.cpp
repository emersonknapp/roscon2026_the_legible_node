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

RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_map_outlier_filter::Node)

namespace occupancy_grid_map_outlier_filter
{

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("OccupancyGridMapOutlierFilter", options)
{
  // Configuration
  params_.map_frame = declare_parameter<std::string>("map_frame", "map");
  params_.base_link_frame = declare_parameter<std::string>("base_link_frame", "base_link");
  params_.cost_threshold = declare_parameter<int>("cost_threshold", 45);
  params_.use_radius_search_2d_filter = declare_parameter<bool>("use_radius_search_2d_filter", true);

  // Communications interface
  pub_pointcloud_ = create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::QoS{5}.reliable());
  sub_occupancy_grid_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid_map", rclcpp::QoS{1}, [this](OccupancyGrid::ConstSharedPtr m) {
      sync_pt_occupancy_grid_.add(m);
    });
  sub_pointcloud_ = create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(), [this](PointCloud2::ConstSharedPtr m) {
      sync_pt_pointcloud_.add(m);
    });

  // Node extensions
  radius_search_mixin_ = std::make_optional<RadiusSearch2dFilterNodeMixin>(*get_node_parameters_interface());
  radius_search_2d_filter_ = std::make_optional<RadiusSearch2dFilter>(radius_search_mixin_->config);
  tf2_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_, *this);

  // Implementation
  sync_ = std::make_shared<Sync>(SyncPolicy(5), sync_pt_occupancy_grid_, sync_pt_pointcloud_);
  sync_->registerCallback(std::bind(&Node::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void Node::sync_callback(const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  auto result = filterPipeline(
    input_ogm, input_pc, tf2_, radius_search_2d_filter_, params_.base_link_frame, params_.cost_threshold);
  if (result) {
    pub_pointcloud_->publish(std::move(result));
  }
}

}  // namespace occupancy_grid_map_outlier_filter
