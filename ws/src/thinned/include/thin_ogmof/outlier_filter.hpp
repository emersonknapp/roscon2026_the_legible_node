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

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/search/kdtree.h"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"

namespace occupancy_grid_map_outlier_filter
{

using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::PointCloud2;

/// @brief
/// @param input
/// @param tf2
/// @param target_frame
/// @param output
/// @return
bool transformPointcloud(
  const PointCloud2 & input, const tf2_ros::Buffer & tf2, const std::string & target_frame, PointCloud2 & output);

/// @brief
/// @param tf2
/// @param target_frame_id
/// @param src_frame_id
/// @param time
/// @return
geometry_msgs::msg::PoseStamped getPoseStamped(
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame_id,
  const std::string & src_frame_id,
  const rclcpp::Time & time);

/// @brief
/// @param map
/// @param x
/// @param y
/// @return
std::optional<char> getCost(const OccupancyGrid & map, const double & x, const double & y);

/// @brief
class RadiusSearch2dFilter
{
public:
  using Pose = geometry_msgs::msg::Pose;

  struct Config
  {
    float search_radius;
    float min_points_and_distance_ratio;
    int min_points;
    int max_points;
    uint32_t max_filter_points_nb;
  };

  explicit RadiusSearch2dFilter(Config config);
  void filter(const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier);
  void filter(
    const PointCloud2 & high_conf_xyz_cloud,
    const PointCloud2 & low_conf_xyz_cloud,
    const Pose & pose,
    PointCloud2 & output,
    PointCloud2 & outlier);

private:
  Config config_;
  pcl::search::Search<pcl::PointXY>::Ptr kd_tree_;
};

/// @brief
class RadiusSearch2dFilterNodeMixin
{
public:
  explicit RadiusSearch2dFilterNodeMixin(rclcpp::node_interfaces::NodeParametersInterface & node_params);
  RadiusSearch2dFilter::Config config;
};

/// @brief
/// @param input
/// @param output
void initializePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief
/// @param input
/// @param output
void finalizePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief
/// @param output
/// @param input
void concatPointCloud2(PointCloud2 & output, const PointCloud2 & input);

/// @brief
/// @param input_pc
/// @param front_pc
/// @param behind_pc
void splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc);

/// @brief
/// @param occupancy_grid_map
/// @param pointcloud
/// @param high_confidence
/// @param low_confidence
/// @param out_ogm
void filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map,
  const PointCloud2 & pointcloud,
  PointCloud2 & high_confidence,
  PointCloud2 & low_confidence,
  PointCloud2 & out_ogm);

/// @brief
/// @param input_ogm
/// @param input_pc
/// @param tf2
/// @return
std::unique_ptr<PointCloud2> filterPipeline(
  const OccupancyGrid::ConstSharedPtr & input_ogm,
  const PointCloud2::ConstSharedPtr & input_pc,
  const std::shared_ptr<tf2_ros::Buffer> tf2,
  std::optional<RadiusSearch2dFilter> radius_search,
  const std::string & base_link_frame);

}  // namespace occupancy_grid_map_outlier_filter
