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

#include "outlier_filter/point_cloud_util.hpp"

#include <cstring>
#include <string>

#include "pcl_conversions/pcl_conversions.h"

namespace thin_occupancy_grid_map_outlier_filter::detail
{

int fieldOffset(const PointCloud2 & cloud, const std::string & name)
{
  return cloud.fields[pcl::getFieldIndex(cloud, name)].offset;
}

float readFloat(const PointCloud2 & cloud, size_t point_offset, int field_offset)
{
  float value;
  std::memcpy(&value, &cloud.data[point_offset + field_offset], sizeof(float));
  return value;
}

void adoptLayout(const PointCloud2 & proto, PointCloud2 & out)
{
  out.header = proto.header;
  out.fields = proto.fields;
  out.point_step = proto.point_step;
  out.height = proto.height;
  out.is_bigendian = proto.is_bigendian;
  out.is_dense = proto.is_dense;
  out.width = out.data.size() / out.point_step / out.height;
  out.row_step = out.data.size() / out.height;
}

PointCloud2 emptyLike(const PointCloud2 & proto)
{
  PointCloud2 out;
  out.point_step = proto.point_step;
  out.data.reserve(proto.data.size());
  return out;
}

void appendPoint(PointCloud2 & out, const PointCloud2 & src, size_t src_offset)
{
  out.data.insert(out.data.end(), src.data.begin() + src_offset, src.data.begin() + src_offset + src.point_step);
}

}  // namespace thin_occupancy_grid_map_outlier_filter::detail
