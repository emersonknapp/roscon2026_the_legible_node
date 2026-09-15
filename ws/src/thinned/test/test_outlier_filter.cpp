// Copyright 2026 Alistair English, Emerson Knapp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The point of the Thin Node pattern: the algorithm is a plain library, so it
// tests with trivial data and no ROS node, executor, or TF wiring.

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "thin_ogmof/outlier_filter.hpp"

namespace occupancy_grid_map_outlier_filter
{
namespace
{

// Build a minimal XYZ float32 cloud from a list of points.
PointCloud2 makeXyzCloud(const std::vector<std::array<float, 3>> & points)
{
  PointCloud2 cloud;
  for (const char * name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = cloud.fields.size() * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.point_step = 3 * sizeof(float);
  cloud.height = 1;
  cloud.width = points.size();
  cloud.data.resize(points.size() * cloud.point_step);
  for (size_t i = 0; i < points.size(); ++i) {
    std::memcpy(&cloud.data[i * cloud.point_step], points[i].data(), cloud.point_step);
  }
  return cloud;
}

float xAt(const PointCloud2 & cloud, size_t i)
{
  float x;
  std::memcpy(&x, &cloud.data[i * cloud.point_step], sizeof(float));
  return x;
}

}  // namespace

TEST(GetCost, ReadsCellUnderPoint)
{
  // 2x2 grid, 1m cells, origin at (0, 0). Costs laid out row-major: y * width + x.
  OccupancyGrid map;
  map.info.resolution = 1.0;
  map.info.width = 2;
  map.info.height = 2;
  map.data = {10, 20, 30, 40};

  EXPECT_EQ(getCost(map, 0.5, 0.5), 10);  // cell (0, 0)
  EXPECT_EQ(getCost(map, 1.5, 0.5), 20);  // cell (1, 0)
  EXPECT_EQ(getCost(map, 0.5, 1.5), 30);  // cell (0, 1)
}

TEST(GetCost, ReturnsNulloptOutsideMap)
{
  OccupancyGrid map;
  map.info.resolution = 1.0;
  map.info.width = 2;
  map.info.height = 2;
  map.data = {10, 20, 30, 40};

  EXPECT_FALSE(getCost(map, 5.0, 5.0).has_value());
}

TEST(SplitPointCloudFrontBack, PartitionsOnSignOfX)
{
  auto input = std::make_shared<PointCloud2>(makeXyzCloud({
    {{1.0f, 0.0f, 0.0f}},  // front
    {{-1.0f, 0.0f, 0.0f}},  // behind
    {{2.0f, 0.0f, 0.0f}},  // front
  }));

  PointCloud2 front;
  PointCloud2 behind;
  initializePointCloud2(*input, front);
  initializePointCloud2(*input, behind);

  splitPointCloudFrontBack(input, front, behind);

  ASSERT_EQ(front.data.size() / input->point_step, 2u);
  ASSERT_EQ(behind.data.size() / input->point_step, 1u);
  EXPECT_FLOAT_EQ(xAt(front, 0), 1.0f);
  EXPECT_FLOAT_EQ(xAt(front, 1), 2.0f);
  EXPECT_FLOAT_EQ(xAt(behind, 0), -1.0f);
}

TEST(FilterPipeline, PassesHighConfidencePointThrough)
{
  // A bare TF buffer with a static identity map<->base_link transform. No node needed.
  auto tf2 = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "base_link";
  identity.transform.rotation.w = 1.0;
  tf2->setTransform(identity, "test", /*is_static=*/true);

  // One point in front of the vehicle, over an all-occupied map, so it lands high-confidence.
  auto pc = std::make_shared<PointCloud2>(makeXyzCloud({{{1.0f, 0.5f, 0.0f}}}));
  pc->header.frame_id = "base_link";

  auto ogm = std::make_shared<OccupancyGrid>();
  ogm->header.frame_id = "map";
  ogm->info.resolution = 1.0;
  ogm->info.width = 4;
  ogm->info.height = 4;
  ogm->data.assign(ogm->info.width * ogm->info.height, 100);  // every cell above threshold

  auto result = filterPipeline(ogm, pc, tf2, /*radius_search=*/std::nullopt, "base_link", /*cost_threshold=*/45);

  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->width, 1u);
  EXPECT_FLOAT_EQ(xAt(*result, 0), 1.0f);
}

}  // namespace occupancy_grid_map_outlier_filter
