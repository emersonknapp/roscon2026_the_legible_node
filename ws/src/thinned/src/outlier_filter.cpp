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
#include <memory>
#include <string>
#include <utility>
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

void initializePointCloud2(const PointCloud2 & input, PointCloud2 & output)
{
  output.point_step = input.point_step;
  output.data.resize(input.data.size());
}

void finalizePointCloud2(const PointCloud2 & input, PointCloud2 & output)
{
  output.header = input.header;
  output.point_step = input.point_step;
  output.fields = input.fields;
  output.height = input.height;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  output.width = output.data.size() / output.point_step / output.height;
  output.row_step = output.data.size() / output.height;
}

void concatPointCloud2(PointCloud2 & output, const PointCloud2 & input)
{
  size_t output_size = output.data.size();
  output.data.resize(output.data.size() + input.data.size());
  std::memcpy(&output.data[output_size], &input.data[0], input.data.size());
}

void splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc)
{
  int x_offset = input_pc->fields[pcl::getFieldIndex(*input_pc, "x")].offset;
  int point_step = input_pc->point_step;
  size_t front_count = 0;
  size_t behind_count = 0;

  for (size_t global_offset = 0; global_offset < input_pc->data.size(); global_offset += point_step) {
    float x;
    std::memcpy(&x, &input_pc->data[global_offset + x_offset], sizeof(float));
    if (x < 0.0) {
      std::memcpy(&behind_pc.data[behind_count * point_step], &input_pc->data[global_offset], input_pc->point_step);
      behind_count++;
    } else {
      std::memcpy(&front_pc.data[front_count * point_step], &input_pc->data[global_offset], input_pc->point_step);
      front_count++;
    }
  }
  front_pc.data.resize(front_count * point_step);
  behind_pc.data.resize(behind_count * point_step);
}

void filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map,
  const PointCloud2 & pointcloud,
  PointCloud2 & high_confidence,
  PointCloud2 & low_confidence,
  PointCloud2 & out_ogm);

std::unique_ptr<PointCloud2> filterPipeline(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & input_ogm,
  const PointCloud2::ConstSharedPtr & input_pc,
  const std::shared_ptr<tf2_ros::Buffer> tf2_buf,
  std::optional<RadiusSearch2dFilter> radius_search,
  const std::string & base_link_frame)
{
  // Transform to occupancy grid map frame

  PointCloud2 input_behind_pc{};
  PointCloud2 input_front_pc{};
  initializePointCloud2(*input_pc, input_front_pc);
  initializePointCloud2(*input_pc, input_behind_pc);
  // Split pointcloud into front and behind of the vehicle to reduce the calculation cost
  splitPointCloudFrontBack(input_pc, input_front_pc, input_behind_pc);
  finalizePointCloud2(*input_pc, input_front_pc);
  finalizePointCloud2(*input_pc, input_behind_pc);

  PointCloud2 ogm_frame_pc{};
  PointCloud2 ogm_frame_input_behind_pc{};
  {  // transform pointclouds
    if (
      !transformPointcloud(input_front_pc, *tf2_buf, input_ogm->header.frame_id, ogm_frame_pc) ||
      !transformPointcloud(input_behind_pc, *tf2_buf, input_ogm->header.frame_id, ogm_frame_input_behind_pc))
    {
      return nullptr;
    }
  }

  // Occupancy grid map based filter
  PointCloud2 high_confidence_pc{};
  PointCloud2 low_confidence_pc{};
  PointCloud2 out_ogm_pc{};
  initializePointCloud2(ogm_frame_pc, high_confidence_pc);
  initializePointCloud2(ogm_frame_pc, low_confidence_pc);
  initializePointCloud2(ogm_frame_pc, out_ogm_pc);
  // split front pointcloud into high and low confidence and out of map pointcloud
  filterByOccupancyGridMap(*input_ogm, ogm_frame_pc, high_confidence_pc, low_confidence_pc, out_ogm_pc);
  // Apply Radius search 2d filter for low confidence pointcloud
  PointCloud2 filtered_low_confidence_pc{};
  PointCloud2 outlier_pc{};
  initializePointCloud2(low_confidence_pc, outlier_pc);
  initializePointCloud2(low_confidence_pc, filtered_low_confidence_pc);

  if (radius_search.has_value()) {
    auto pc_frame_pose_stamped =
      getPoseStamped(*tf2_buf, input_ogm->header.frame_id, input_pc->header.frame_id, input_ogm->header.stamp);
    radius_search->filter(
      high_confidence_pc, low_confidence_pc, pc_frame_pose_stamped.pose, filtered_low_confidence_pc, outlier_pc);
  } else {
    std::memcpy(&outlier_pc.data[0], &low_confidence_pc.data[0], low_confidence_pc.data.size());
    outlier_pc.data.resize(low_confidence_pc.data.size());
  }

  // Concatenate high confidence pointcloud from occupancy grid map and non-outlier pointcloud
  PointCloud2 ogm_frame_filtered_pc{};
  concatPointCloud2(ogm_frame_filtered_pc, high_confidence_pc);
  concatPointCloud2(ogm_frame_filtered_pc, filtered_low_confidence_pc);
  concatPointCloud2(ogm_frame_filtered_pc, out_ogm_pc);
  concatPointCloud2(ogm_frame_filtered_pc, ogm_frame_input_behind_pc);
  finalizePointCloud2(ogm_frame_pc, ogm_frame_filtered_pc);

  auto base_link_frame_filtered_pc_ptr = std::make_unique<PointCloud2>();
  {
    ogm_frame_filtered_pc.header = ogm_frame_pc.header;
    if (!transformPointcloud(ogm_frame_filtered_pc, *tf2_buf, base_link_frame, *base_link_frame_filtered_pc_ptr)) {
      return nullptr;
    }
  }

  return std::move(base_link_frame_filtered_pc_ptr);
}

}  // namespace occupancy_grid_map_outlier_filter
