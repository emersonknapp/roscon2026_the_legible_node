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

#include "outlier_filter/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "outlier_filter/occupancy_grid.hpp"
#include "outlier_filter/point_cloud_util.hpp"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl_ros/transforms.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{

SplitClouds splitFrontBack(const PointCloud2 & input)
{
  SplitClouds result{detail::emptyLike(input), detail::emptyLike(input)};
  const int x_offset = detail::fieldOffset(input, "x");

  for (size_t offset = 0; offset < input.data.size(); offset += input.point_step) {
    const float x = detail::readFloat(input, offset, x_offset);
    detail::appendPoint(x < 0.0F ? result.behind : result.front, input, offset);
  }

  detail::adoptLayout(input, result.front);
  detail::adoptLayout(input, result.behind);
  return result;
}

PointCloud2 transformPointCloud(
  const PointCloud2 & input, const Eigen::Matrix4f & transform, const std::string & target_frame)
{
  PointCloud2 output;
  pcl_ros::transformPointCloud(transform, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return output;
}

ClassifiedClouds classifyByOccupancyGridMap(const OccupancyGrid & map, const PointCloud2 & cloud, int cost_threshold)
{
  ClassifiedClouds result{detail::emptyLike(cloud), detail::emptyLike(cloud), detail::emptyLike(cloud)};
  const int x_offset = detail::fieldOffset(cloud, "x");
  const int y_offset = detail::fieldOffset(cloud, "y");

  for (size_t offset = 0; offset < cloud.data.size(); offset += cloud.point_step) {
    const float x = detail::readFloat(cloud, offset, x_offset);
    const float y = detail::readFloat(cloud, offset, y_offset);
    const auto cost = detail::costAt(map, x, y);

    if (!cost) {
      detail::appendPoint(result.out_of_map, cloud, offset);
    } else if (cost_threshold < *cost) {
      detail::appendPoint(result.high_confidence, cloud, offset);
    } else {
      detail::appendPoint(result.low_confidence, cloud, offset);
    }
  }

  detail::adoptLayout(cloud, result.high_confidence);
  detail::adoptLayout(cloud, result.low_confidence);
  detail::adoptLayout(cloud, result.out_of_map);
  return result;
}

Pose sensorPoseInMap(const Eigen::Matrix4f & sensor_to_map)
{
  // The sensor origin expressed in the map frame is the translation column.
  Pose pose;
  pose.position.x = sensor_to_map(0, 3);
  pose.position.y = sensor_to_map(1, 3);
  pose.position.z = sensor_to_map(2, 3);
  return pose;
}

RadiusFilterResult radiusSearch2dFilter(
  const PointCloud2 & high_confidence,
  const PointCloud2 & low_confidence,
  const Pose & sensor_pose,
  const RadiusSearch2dParams & params)
{
  RadiusFilterResult result{detail::emptyLike(low_confidence), detail::emptyLike(low_confidence)};

  // Guard against pathological loads: skip filtering, keep the candidates as-is.
  if (low_confidence.width > params.max_filter_points_nb) {
    result.kept = low_confidence;
    detail::adoptLayout(low_confidence, result.kept);
    detail::adoptLayout(low_confidence, result.outliers);
    return result;
  }

  const int x_offset = detail::fieldOffset(low_confidence, "x");
  const int y_offset = detail::fieldOffset(low_confidence, "y");
  const int point_step = low_confidence.point_step;

  // Build the 2D search set: low-confidence points first, then high-confidence
  // points as neighbourhood support. Only the low-confidence prefix is judged.
  auto xy_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXY>>();
  xy_cloud->points.resize(low_confidence.width + high_confidence.width);
  for (size_t i = 0; i < low_confidence.width; ++i) {
    xy_cloud->points[i].x = detail::readFloat(low_confidence, i * point_step, x_offset);
    xy_cloud->points[i].y = detail::readFloat(low_confidence, i * point_step, y_offset);
  }
  for (size_t i = 0; i < high_confidence.width; ++i) {
    xy_cloud->points[low_confidence.width + i].x = detail::readFloat(high_confidence, i * point_step, x_offset);
    xy_cloud->points[low_confidence.width + i].y = detail::readFloat(high_confidence, i * point_step, y_offset);
  }

  auto kd_tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
  kd_tree->setInputCloud(xy_cloud);

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  for (size_t i = 0; i < low_confidence.width; ++i) {
    const float distance =
      std::hypot(xy_cloud->points[i].x - sensor_pose.position.x, xy_cloud->points[i].y - sensor_pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(std::lround(params.min_points_and_distance_ratio / distance)), params.min_points),
      params.max_points);
    const int points_num = kd_tree->radiusSearch(i, params.search_radius, k_indices, k_distances, min_points_threshold);

    detail::appendPoint(
      min_points_threshold <= points_num ? result.kept : result.outliers, low_confidence, i * point_step);
  }

  detail::adoptLayout(low_confidence, result.kept);
  detail::adoptLayout(low_confidence, result.outliers);
  return result;
}

PointCloud2 concatenate(const std::vector<const PointCloud2 *> & clouds)
{
  PointCloud2 output;
  if (clouds.empty()) {
    return output;
  }

  size_t total = 0;
  for (const auto * cloud : clouds) {
    total += cloud->data.size();
  }
  output.data.reserve(total);
  for (const auto * cloud : clouds) {
    output.data.insert(output.data.end(), cloud->data.begin(), cloud->data.end());
  }

  detail::adoptLayout(*clouds.front(), output);
  return output;
}

}  // namespace thin_occupancy_grid_map_outlier_filter
