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

/// @brief Transform a pointcloud into target_frame using the given TF buffer.
/// @param output Transformed cloud, written on success.
/// @return False if the transform lookup fails.
bool transformPointcloud(
  const PointCloud2 & input, const tf2_ros::Buffer & tf2, const std::string & target_frame, PointCloud2 & output);

/// @brief Look up the pose of src_frame_id in target_frame_id at the given time.
geometry_msgs::msg::PoseStamped getPoseStamped(
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame_id,
  const std::string & src_frame_id,
  const rclcpp::Time & time);

/// @brief Sample the occupancy grid cost at world coordinate (x, y).
/// @return The cell cost, or nullopt if (x, y) falls outside the map.
std::optional<char> getCost(const OccupancyGrid & map, const double & x, const double & y);

/// @brief Rejects sparse points by 2D radius neighbor search over a kd-tree.
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

  /// @brief Split input into surviving points (output) and rejected points (outlier).
  void filter(const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier);

  /// @brief As above, but seeds the neighbor search with high-confidence points so
  /// low-confidence points near real obstacles survive. Only low-confidence points are classified.
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

/// @brief Declares this filter's parameters on a node and holds the resulting Config.
class RadiusSearch2dFilterNodeMixin
{
public:
  explicit RadiusSearch2dFilterNodeMixin(rclcpp::node_interfaces::NodeParametersInterface & node_params);
  RadiusSearch2dFilter::Config config;
};

/// @brief Size output to match input's layout and point count, ready to be filled.
void initializePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief Copy input's metadata onto output and recompute its width/row_step from data size.
void finalizePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief Append input's points onto the end of output.
void concatPointCloud2(PointCloud2 & output, const PointCloud2 & input);

/// @brief Partition input_pc into points ahead of (front_pc) and behind (behind_pc) the vehicle.
void splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc);

/// @brief Bucket each point by its occupancy grid cost into high/low confidence and out-of-map clouds.
void filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map,
  const PointCloud2 & pointcloud,
  const int cost_threshold,
  PointCloud2 & high_confidence,
  PointCloud2 & low_confidence,
  PointCloud2 & out_ogm);

/// @brief Full outlier-filter pipeline: split, transform, classify, radius-filter, recombine.
/// @param radius_search Optional radius filter; when absent, low-confidence points are dropped.
/// @return Filtered cloud in base_link_frame, or nullptr if a transform fails.
std::unique_ptr<PointCloud2> filterPipeline(
  const OccupancyGrid::ConstSharedPtr & input_ogm,
  const PointCloud2::ConstSharedPtr & input_pc,
  const std::shared_ptr<tf2_ros::Buffer> tf2,
  std::optional<RadiusSearch2dFilter> radius_search,
  const std::string & base_link_frame,
  const int cost_threshold);

}  // namespace occupancy_grid_map_outlier_filter
