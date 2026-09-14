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

// The data model for the outlier filter pipeline: the configuration/input
// structs and the intermediate/output structs. This is the vocabulary the rest
// of the pipeline is written in -- read it first and the prose in
// outlier_filter.cpp follows naturally. No logic lives here.

#pragma once

#include <cstdint>
#include <Eigen/Core>  // NOLINT(build/include_order)
#include <optional>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{
using sensor_msgs::msg::PointCloud2;

// ---------------------------------------------------------------------------
// Configuration & inputs
// ---------------------------------------------------------------------------

// Parameters for the radius-search 2D outlier filter. Wrapped in std::optional
// at the call site: absent => the filter is disabled.
struct RadiusSearch2dParams
{
  float search_radius;
  float min_points_and_distance_ratio;
  int min_points;
  int max_points;
  uint32_t max_filter_points_nb;
};

struct OutlierFilterParams
{
  int cost_threshold;
  std::optional<RadiusSearch2dParams> radius_search;  // std::nullopt => disabled
};

// All frame transforms the pipeline needs, resolved by the caller (from TF) at
// the right timestamps. Keeping TF out of the core keeps it pure and testable.
// The map frame itself is taken from the OccupancyGrid header, so it is not
// duplicated here.
//   sensor_to_map    : input_pc frame            -> occupancy-grid-map frame
//   map_to_base_link : occupancy-grid-map frame  -> base_link frame
struct FrameTransforms
{
  Eigen::Matrix4f sensor_to_map;
  Eigen::Matrix4f map_to_base_link;
  std::string base_link_frame;
};

// ---------------------------------------------------------------------------
// Intermediate results (named so the top-level pipeline reads like prose)
// ---------------------------------------------------------------------------

struct SplitClouds
{
  PointCloud2 front;  // x >= 0
  PointCloud2 behind;  // x <  0
};

struct ClassifiedClouds
{
  PointCloud2 high_confidence;  // grid cost > threshold  (trusted obstacle)
  PointCloud2 low_confidence;  // grid cost <= threshold (outlier candidate)
  PointCloud2 out_of_map;  // no cost available      (passed through)
};

struct RadiusFilterResult
{
  PointCloud2 kept;  // survived the radius search
  PointCloud2 outliers;  // rejected
};

// The intermediate clouds the node otherwise publishes on its debug topics,
// transformed into the base_link frame. Only produced when requested.
struct DebugClouds
{
  PointCloud2 high_confidence;  // trusted obstacles
  PointCloud2 low_confidence_kept;  // candidates that survived the radius filter
  PointCloud2 outliers;  // rejected candidates
};

// Top-level output: the filtered cloud (base_link frame) plus, optionally, the
// debug clouds. `debug` is std::nullopt unless debug output was requested, so
// callers pay for the extra transforms only when they want them.
struct OutlierFilterResult
{
  PointCloud2 pointcloud;
  std::optional<DebugClouds> debug;
};

}  // namespace thin_occupancy_grid_map_outlier_filter
