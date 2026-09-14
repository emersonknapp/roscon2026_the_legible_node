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

#include "outlier_filter/outlier_filter.hpp"

#include "outlier_filter/pipeline.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{

// Top-level entry point: pure routing, no branching logic beyond the one
// optional filter toggle.
OutlierFilterResult filterOccupancyGridMapOutliers(
  const PointCloud2 & input_pc,
  const OccupancyGrid & map,
  const FrameTransforms & tf,
  const OutlierFilterParams & params,
  bool with_debug)
{
  // Split into front/behind so only the front is filtered (cost reduction).
  const SplitClouds split = splitFrontBack(input_pc);

  // Bring both halves into the occupancy-grid-map frame.
  const PointCloud2 front_in_map = transformPointCloud(split.front, tf.sensor_to_map, map.header.frame_id);
  const PointCloud2 behind_in_map = transformPointCloud(split.behind, tf.sensor_to_map, map.header.frame_id);

  // Classify front points against the map: trusted / candidate / unjudgeable.
  const ClassifiedClouds classified = classifyByOccupancyGridMap(map, front_in_map, params.cost_threshold);

  // Radius-filter the low-confidence candidates (or, if disabled, drop them all).
  const RadiusFilterResult low = params.radius_search ? radiusSearch2dFilter(
                                                          classified.high_confidence,
                                                          classified.low_confidence,
                                                          sensorPoseInMap(tf.sensor_to_map),
                                                          *params.radius_search)
                                                      : RadiusFilterResult{PointCloud2{}, classified.low_confidence};

  // Reassemble everything we keep (outliers excluded), still in the map frame.
  const PointCloud2 filtered_in_map =
    concatenate({&classified.high_confidence, &low.kept, &classified.out_of_map, &behind_in_map});

  // Back to base_link for downstream consumers.
  OutlierFilterResult result;
  result.pointcloud = transformPointCloud(filtered_in_map, tf.map_to_base_link, tf.base_link_frame);

  // Optionally hand back the intermediate clouds, also in base_link.
  if (with_debug) {
    result.debug = DebugClouds{
      transformPointCloud(classified.high_confidence, tf.map_to_base_link, tf.base_link_frame),
      transformPointCloud(low.kept, tf.map_to_base_link, tf.base_link_frame),
      transformPointCloud(low.outliers, tf.map_to_base_link, tf.base_link_frame)};
  }

  return result;
}

}  // namespace thin_occupancy_grid_map_outlier_filter
