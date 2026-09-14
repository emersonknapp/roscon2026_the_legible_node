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

// The one occupancy-grid-specific accessor: looking up the cost at a world
// coordinate. Kept apart from the generic PointCloud2 plumbing so map-domain
// logic lives in an obviously named place.

#pragma once

#include <cstdint>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace thin_occupancy_grid_map_outlier_filter::detail
{
using nav_msgs::msg::OccupancyGrid;

// Occupancy cost at world (x, y), or nullopt if the point is outside the map.
std::optional<int8_t> costAt(const OccupancyGrid & map, double x, double y);

}  // namespace thin_occupancy_grid_map_outlier_filter::detail
