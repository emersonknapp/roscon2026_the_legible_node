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

#include "outlier_filter/occupancy_grid.hpp"

#include <cmath>

namespace thin_occupancy_grid_map_outlier_filter::detail
{

std::optional<int8_t> costAt(const OccupancyGrid & map, double x, double y)
{
  const auto & origin = map.info.origin.position;
  const double resolution = map.info.resolution;
  const double min_x = origin.x;
  const double max_x = origin.x + map.info.width * resolution;
  const double min_y = origin.y;
  const double max_y = origin.y + map.info.height * resolution;

  if (min_x < x && x < max_x && min_y < y && y < max_y) {
    const auto cell_x = static_cast<unsigned int>(std::floor((x - origin.x) / resolution));
    const auto cell_y = static_cast<unsigned int>(std::floor((y - origin.y) / resolution));
    return map.data.at(cell_y * map.info.width + cell_x);
  }
  return std::nullopt;
}

}  // namespace thin_occupancy_grid_map_outlier_filter::detail
