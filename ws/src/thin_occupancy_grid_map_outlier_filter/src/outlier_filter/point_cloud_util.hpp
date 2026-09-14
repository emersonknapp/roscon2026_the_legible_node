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

// Generic PointCloud2 byte plumbing. These helpers know how to read fields out
// of a serialized cloud and how to copy points/layout between clouds, but carry
// no knowledge of this filter's domain -- they would work for any PointCloud2.

#pragma once

#include <cstddef>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace thin_occupancy_grid_map_outlier_filter::detail
{
using sensor_msgs::msg::PointCloud2;

// Byte offset of a named field (e.g. "x") within a single point.
int fieldOffset(const PointCloud2 & cloud, const std::string & name);

// Read a float field out of the point starting at `point_offset` bytes.
float readFloat(const PointCloud2 & cloud, size_t point_offset, int field_offset);

// Copy the metadata (fields, layout, header) of `proto` onto `out` and recompute
// the width/row_step from however many bytes `out` currently holds.
void adoptLayout(const PointCloud2 & proto, PointCloud2 & out);

// Create an empty cloud that mirrors `proto`'s layout, reserving worst-case bytes.
PointCloud2 emptyLike(const PointCloud2 & proto);

// Append the single point at `src_offset` bytes of `src` to the end of `out`.
void appendPoint(PointCloud2 & out, const PointCloud2 & src, size_t src_offset);

}  // namespace thin_occupancy_grid_map_outlier_filter::detail
