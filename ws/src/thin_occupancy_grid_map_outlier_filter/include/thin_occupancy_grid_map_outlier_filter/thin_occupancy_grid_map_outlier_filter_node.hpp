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

#pragma once

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "thin_occupancy_grid_map_outlier_filter/visibility_control.hpp"

namespace thin_occupancy_grid_map_outlier_filter
{

THIN_OGM_OUTLIER_FILTER_PUBLIC
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr create_node(const rclcpp::NodeOptions & options);

}  // namespace thin_occupancy_grid_map_outlier_filter
