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

// Public entry point of the functional core. Given a cloud, a map, the frame
// transforms and the parameters, produce the filtered cloud (and, optionally,
// the debug clouds). This is the only header a caller needs to include.

#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "outlier_filter/types.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{
using nav_msgs::msg::OccupancyGrid;

OutlierFilterResult filterOccupancyGridMapOutliers(
  const PointCloud2 & input_pc,
  const OccupancyGrid & map,
  const FrameTransforms & tf,
  const OutlierFilterParams & params,
  bool with_debug = false);

}  // namespace thin_occupancy_grid_map_outlier_filter
