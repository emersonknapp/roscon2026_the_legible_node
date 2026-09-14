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

#include "node.hpp"  // NOLINT(build/include_subdir)

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "thin_occupancy_grid_map_outlier_filter/thin_occupancy_grid_map_outlier_filter_node.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(thin_occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent)

namespace thin_occupancy_grid_map_outlier_filter
{

OccupancyGridMapOutlierFilterComponent::OccupancyGridMapOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Node("OccupancyGridMapOutlierFilter", options)
{
  base_link_frame_ = declare_parameter<std::string>("base_link_frame");
  params_.cost_threshold = declare_parameter<int>("cost_threshold");
  const auto use_radius_search_2d_filter = declare_parameter<bool>("use_radius_search_2d_filter");
  enable_debugger_ = declare_parameter<bool>("enable_debugger");

  if (use_radius_search_2d_filter) {
    RadiusSearch2dParams radius{};
    radius.search_radius = declare_parameter<float>("radius_search_2d_filter.search_radius");
    radius.min_points_and_distance_ratio =
      declare_parameter<float>("radius_search_2d_filter.min_points_and_distance_ratio");
    radius.min_points = declare_parameter<int>("radius_search_2d_filter.min_points");
    radius.max_points = declare_parameter<int>("radius_search_2d_filter.max_points");
    radius.max_filter_points_nb =
      static_cast<uint32_t>(declare_parameter<int>("radius_search_2d_filter.max_filter_points_nb"));
    params_.radius_search = radius;
  }

  tf2_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);

  pointcloud_sub_.subscribe(this, "~/input/pointcloud", rclcpp::SensorDataQoS());
  occupancy_grid_map_sub_.subscribe(this, "~/input/occupancy_grid_map", rclcpp::QoS{1});
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(5), occupancy_grid_map_sub_, pointcloud_sub_);
  sync_ptr_->registerCallback(std::bind(
    &OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2,
    this,
    std::placeholders::_1,
    std::placeholders::_2));

  /**
   * To avoid data loss and simplify the operation, the hard coded QoS setting as Reliable is used
   * for the publisher of obstacle_segmentation/pointcloud. If there is a clear distinction between
   * data collection and autonomous driving modes, PublisherOptions with
   * QosOverridingOptions::with_default_policies should be good options instead of hard-coding QoS
   * setting.
   */
  pointcloud_pub_ = create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::QoS{5}.reliable());

  if (enable_debugger_) {
    high_confidence_pub_ =
      create_publisher<PointCloud2>("~/output/debug/high_confidence/pointcloud", rclcpp::SensorDataQoS());
    low_confidence_pub_ =
      create_publisher<PointCloud2>("~/output/debug/low_confidence/pointcloud", rclcpp::SensorDataQoS());
    outlier_pub_ = create_publisher<PointCloud2>("~/output/debug/outlier/pointcloud", rclcpp::SensorDataQoS());
  }
}

std::optional<Eigen::Matrix4f> OccupancyGridMapOutlierFilterComponent::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    const auto tf_stamped =
      tf2_->lookupTransform(target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    return tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return std::nullopt;
  }
}

void OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2(
  const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  const auto & map_frame = input_ogm->header.frame_id;
  const rclcpp::Time stamp{input_ogm->header.stamp};

  const auto sensor_to_map = lookupTransform(map_frame, input_pc->header.frame_id, stamp);
  const auto map_to_base_link = lookupTransform(base_link_frame_, map_frame, stamp);
  if (!sensor_to_map || !map_to_base_link) {
    return;
  }

  FrameTransforms tf;
  tf.sensor_to_map = *sensor_to_map;
  tf.map_to_base_link = *map_to_base_link;
  tf.base_link_frame = base_link_frame_;

  auto result = filterOccupancyGridMapOutliers(*input_pc, *input_ogm, tf, params_, enable_debugger_);

  pointcloud_pub_->publish(std::make_unique<PointCloud2>(std::move(result.pointcloud)));

  if (result.debug) {
    high_confidence_pub_->publish(std::make_unique<PointCloud2>(std::move(result.debug->high_confidence)));
    low_confidence_pub_->publish(std::make_unique<PointCloud2>(std::move(result.debug->low_confidence_kept)));
    outlier_pub_->publish(std::make_unique<PointCloud2>(std::move(result.debug->outliers)));
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr create_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<OccupancyGridMapOutlierFilterComponent>(options)->get_node_base_interface();
}

}  // namespace thin_occupancy_grid_map_outlier_filter
