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

// The individual pipeline steps, declared in execution order so this header
// reads as a summary of what the filter does. The orchestration that strings
// them together lives in outlier_filter.cpp; the implementations in
// pipeline.cpp.

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "outlier_filter/types.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

// Partition a cloud into front (x >= 0) and behind (x < 0) halves.
SplitClouds splitFrontBack(const PointCloud2 & input);

// Rigid-transform a cloud into `target_frame`, preserving its timestamp.
PointCloud2 transformPointCloud(
  const PointCloud2 & input, const Eigen::Matrix4f & transform, const std::string & target_frame);

// Sort points into trusted / candidate / unjudgeable by their occupancy cost.
ClassifiedClouds classifyByOccupancyGridMap(const OccupancyGrid & map, const PointCloud2 & cloud, int cost_threshold);

// The sensor origin, expressed in the map frame (translation column of the tf).
Pose sensorPoseInMap(const Eigen::Matrix4f & sensor_to_map);

// Radius-search the low-confidence candidates, using the high-confidence points
// as neighbourhood support; split them into kept and outliers.
RadiusFilterResult radiusSearch2dFilter(
  const PointCloud2 & high_confidence,
  const PointCloud2 & low_confidence,
  const Pose & sensor_pose,
  const RadiusSearch2dParams & params);

// Concatenate several clouds (assumed to share a layout) into one.
PointCloud2 concatenate(const std::vector<const PointCloud2 *> & clouds);

}  // namespace thin_occupancy_grid_map_outlier_filter
