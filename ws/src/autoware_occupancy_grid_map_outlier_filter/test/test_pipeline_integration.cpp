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

// The original node has no library seam, so the equivalent of the thin
// filterPipeline test has to run the whole node: a second node publishes the
// inputs and a static transform, both spin on one executor, and we wait for the
// output topic. Compare to the thin package's direct filterPipeline() call.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "autoware_occupancy_grid_map_outlier_filter/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace
{
using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;
using autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent;
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::PointCloud2;

// The node subscribes/publishes relative to its own name, which resolves to these.
constexpr char kInputPc[] = "/OccupancyGridMapOutlierFilter/input/pointcloud";
constexpr char kInputOgm[] = "/OccupancyGridMapOutlierFilter/input/occupancy_grid_map";
constexpr char kOutputPc[] = "/OccupancyGridMapOutlierFilter/output/pointcloud";

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

class PipelineIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

}  // namespace

TEST_F(PipelineIntegrationTest, PassesHighConfidencePointThrough)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"map_frame", "map"},
    {"base_link_frame", "base_link"},
    {"cost_threshold", 45},
    {"use_radius_search_2d_filter", false},
    {"enable_debugger", false},
  });
  auto component = std::make_shared<OccupancyGridMapOutlierFilterComponent>(options);

  auto helper = std::make_shared<rclcpp::Node>("test_helper");

  // Static identity map -> base_link, latched, so the node's TF listener resolves lookups.
  tf2_ros::StaticTransformBroadcaster tf_broadcaster(*helper);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(tf);

  // QoS must match the node's subscriptions (sensor data for the cloud, reliable for the grid).
  auto pc_pub = helper->create_publisher<PointCloud2>(kInputPc, rclcpp::SensorDataQoS());
  auto ogm_pub = helper->create_publisher<OccupancyGrid>(kInputOgm, rclcpp::QoS{1});

  std::mutex mtx;
  std::condition_variable cv;
  PointCloud2 output;
  bool received = false;
  auto out_sub = helper->create_subscription<PointCloud2>(
    kOutputPc, rclcpp::QoS{5}.reliable(), [&](PointCloud2::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(mtx);
        output = *msg;
        received = true;
      }
      cv.notify_one();
    });

  // Matching stamps so the exact-time synchronizer pairs the two messages.
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1;

  auto pc = makeXyzCloud({{{1.0f, 0.5f, 0.0f}}});
  pc.header.frame_id = "base_link";
  pc.header.stamp = stamp;

  OccupancyGrid ogm;
  ogm.header.frame_id = "map";
  ogm.header.stamp = stamp;
  ogm.info.resolution = 1.0;
  ogm.info.width = 4;
  ogm.info.height = 4;
  ogm.data.assign(ogm.info.width * ogm.info.height, 100);  // every cell above threshold

  // Spin on a background thread so the result callback fires the moment the node publishes.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(component);
  exec.add_node(helper);
  std::thread spin_thread([&exec]() { exec.spin(); });

  // Wait for the node's subscriptions and our subscription to connect before publishing.
  // The inputs are volatile, so anything sent before discovery completes is dropped. This
  // polls on the connection state rather than sleeping a fixed "probably long enough" interval.
  const auto connect_deadline = std::chrono::steady_clock::now() + 5s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < connect_deadline &&
         (pc_pub->get_subscription_count() == 0 || ogm_pub->get_subscription_count() == 0 ||
          out_sub->get_publisher_count() == 0))
  {
    std::this_thread::sleep_for(1ms);
  }

  pc_pub->publish(pc);
  ogm_pub->publish(ogm);

  // Block until the callback signals, so the test finishes the instant the result arrives.
  // The 5s is only a failure timeout, not a latency floor on the success path.
  bool got_result = false;
  {
    std::unique_lock<std::mutex> lock(mtx);
    got_result = cv.wait_for(lock, 5s, [&received]() { return received; });
  }

  exec.cancel();
  spin_thread.join();

  ASSERT_TRUE(got_result);
  EXPECT_EQ(output.width, 1u);
}
