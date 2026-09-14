// Copyright 2026 Alistair English, Emerson Knapp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "thin_ogmof/outlier_filter.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "pcl/filters/extract_indices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace occupancy_grid_map_outlier_filter
{

bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame,
  sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped =
      tf2.lookupTransform(target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("occupancy_grid_map_outlier_filter"), clock, 5000, "%s", ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

geometry_msgs::msg::PoseStamped getPoseStamped(
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame_id,
  const std::string & src_frame_id,
  const rclcpp::Time & time)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped = tf2.lookupTransform(target_frame_id, src_frame_id, time, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("occupancy_grid_map_outlier_filter"), clock, 5000, "%s", ex.what());
  }
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = tf_stamped.header;
  pose_stamped.pose.position.x = tf_stamped.transform.translation.x;
  pose_stamped.pose.position.y = tf_stamped.transform.translation.y;
  pose_stamped.pose.position.z = tf_stamped.transform.translation.z;
  pose_stamped.pose.orientation = tf_stamped.transform.rotation;
  return pose_stamped;
}

std::optional<char> getCost(const nav_msgs::msg::OccupancyGrid & map, const double & x, const double & y)
{
  const auto & map_position = map.info.origin.position;
  const auto & map_resolution = map.info.resolution;
  const double map_height_m = map.info.height /* cell size */ * map_resolution;
  const double map_width_m = map.info.width /* cell size */ * map_resolution;
  const double map_min_x = map_position.x;
  const double map_max_x = map_position.x + map_width_m;
  const double map_min_y = map_position.y;
  const double map_max_y = map_position.y + map_height_m;

  if (map_min_x < x && x < map_max_x && map_min_y < y && y < map_max_y) {
    unsigned int map_cell_x{};
    unsigned int map_cell_y{};
    map_cell_x = std::floor((x - map_position.x) / map_resolution);
    map_cell_y = std::floor((y - map_position.y) / map_resolution);
    size_t index = map_cell_y * map.info.width + map_cell_x;
    return map.data.at(index);
  }
  return std::nullopt;
}

RadiusSearch2dFilter::RadiusSearch2dFilter(RadiusSearch2dFilter::Config config)
: config_(config)
, kd_tree_(pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false))
{}

void RadiusSearch2dFilter::filter(
  const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier)
{
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  int point_step = input.point_step;
  int x_offset = input.fields[pcl::getFieldIndex(input, "x")].offset;
  int y_offset = input.fields[pcl::getFieldIndex(input, "y")].offset;
  xy_cloud->points.resize(input.data.size() / point_step);
  for (size_t i = 0; i < input.data.size() / point_step; ++i) {
    std::memcpy(&xy_cloud->points[i].x, &input.data[i * point_step + x_offset], sizeof(float));
    std::memcpy(&xy_cloud->points[i].y, &input.data[i * point_step + y_offset], sizeof(float));
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  size_t output_size = 0;
  size_t outlier_size = 0;
  for (size_t i = 0; i < xy_cloud->points.size(); ++i) {
    const float distance = std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(std::lround(config_.min_points_and_distance_ratio / distance)), config_.min_points),
      config_.max_points);
    const int points_num =
      kd_tree_->radiusSearch(i, config_.search_radius, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      std::memcpy(&output.data[output_size], &input.data[i * point_step], point_step);
      output_size += point_step;
    } else {
      std::memcpy(&outlier.data[outlier_size], &input.data[i * point_step], point_step);
      outlier_size += point_step;
    }
  }
  output.data.resize(output_size);
  outlier.data.resize(outlier_size);
}

void RadiusSearch2dFilter::filter(
  const PointCloud2 & high_conf_xyz_cloud,
  const PointCloud2 & low_conf_xyz_cloud,
  const Pose & pose,
  PointCloud2 & output,
  PointCloud2 & outlier)
{
  // check the limit points number
  if (low_conf_xyz_cloud.width > config_.max_filter_points_nb) {
    RCLCPP_WARN(
      rclcpp::get_logger("OccupancyGridMapOutlierFilterComponent"),
      "Skip outlier filter since too much low_confidence pointcloud!");
    return;
  }
  int x_offset = low_conf_xyz_cloud.fields[pcl::getFieldIndex(low_conf_xyz_cloud, "x")].offset;
  int y_offset = low_conf_xyz_cloud.fields[pcl::getFieldIndex(low_conf_xyz_cloud, "y")].offset;
  int point_step = low_conf_xyz_cloud.point_step;
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(low_conf_xyz_cloud.width + high_conf_xyz_cloud.width);
  for (size_t i = 0; i < low_conf_xyz_cloud.width; ++i) {
    std::memcpy(&xy_cloud->points[i].x, &low_conf_xyz_cloud.data[i * point_step + x_offset], sizeof(float));
    std::memcpy(&xy_cloud->points[i].y, &low_conf_xyz_cloud.data[i * point_step + y_offset], sizeof(float));
  }

  for (size_t i = low_conf_xyz_cloud.width; i < xy_cloud->points.size(); ++i) {
    size_t high_conf_xyz_cloud_index = i - low_conf_xyz_cloud.width;
    std::memcpy(
      &xy_cloud->points[i].x,
      &high_conf_xyz_cloud.data[high_conf_xyz_cloud_index * point_step + x_offset],
      sizeof(float));
    std::memcpy(
      &xy_cloud->points[i].y,
      &high_conf_xyz_cloud.data[high_conf_xyz_cloud_index * point_step + y_offset],
      sizeof(float));
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);

  size_t output_size = 0;
  size_t outlier_size = 0;
  for (size_t i = 0; i < low_conf_xyz_cloud.data.size() / low_conf_xyz_cloud.point_step; ++i) {
    const float distance = std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(std::lround(config_.min_points_and_distance_ratio / distance)), config_.min_points),
      config_.max_points);
    const int points_num =
      kd_tree_->radiusSearch(i, config_.search_radius, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      std::memcpy(
        &output.data[output_size],
        &low_conf_xyz_cloud.data[i * low_conf_xyz_cloud.point_step],
        low_conf_xyz_cloud.point_step);
      output_size += low_conf_xyz_cloud.point_step;
    } else {
      std::memcpy(
        &outlier.data[outlier_size],
        &low_conf_xyz_cloud.data[i * low_conf_xyz_cloud.point_step],
        low_conf_xyz_cloud.point_step);
      outlier_size += low_conf_xyz_cloud.point_step;
    }
  }

  output.data.resize(output_size);
  outlier.data.resize(outlier_size);
}

RadiusSearch2dFilterNodeMixin::RadiusSearch2dFilterNodeMixin(
  rclcpp::node_interfaces::NodeParametersInterface & node_params)
{
  config.search_radius =
    node_params.declare_parameter("radius_search_2d_filter.search_radius", rclcpp::ParameterValue(1.0f)).get<float>();
  config.min_points_and_distance_ratio =
    node_params
      .declare_parameter("radius_search_2d_filter.min_points_and_distance_ratio", rclcpp::ParameterValue(400.0))
      .get<float>();
  config.min_points =
    node_params.declare_parameter("radius_search_2d_filter.min_points", rclcpp::ParameterValue(4)).get<int>();
  config.max_points =
    node_params.declare_parameter("radius_search_2d_filter.max_points", rclcpp::ParameterValue(70)).get<int>();
  config.max_filter_points_nb =
    node_params.declare_parameter("radius_search_2d_filter.max_filter_points_nb", rclcpp::ParameterValue(15000))
      .get<int>();
}

}  // namespace occupancy_grid_map_outlier_filter
