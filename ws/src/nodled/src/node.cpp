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

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "message_filters/pass_through.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "occupancy_grid_map_outlier_filter_base.hpp"  // NOLINT(build/include_subdir)
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "thin_ogmof/outlier_filter.hpp"

namespace occupancy_grid_map_outlier_filter
{

class NodlNode final : public OccupancyGridMapOutlierFilterBase
{
public:
  explicit NodlNode(const rclcpp::NodeOptions & options)
  : OccupancyGridMapOutlierFilterBase(options)
  {
    tf2_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_, *this);

    if (params_.use_radius_search_2d_filter) {
      radius_search_2d_filter_.emplace(RadiusSearch2dFilter::Config{
        static_cast<float>(params_.radius_search_2d_filter.search_radius),
        static_cast<float>(params_.radius_search_2d_filter.min_points_and_distance_ratio),
        static_cast<int>(params_.radius_search_2d_filter.min_points),
        static_cast<int>(params_.radius_search_2d_filter.max_points),
        static_cast<uint32_t>(params_.radius_search_2d_filter.max_filter_points_nb),
      });
    }

    sync_ = std::make_shared<Sync>(SyncPolicy(5), sync_pt_occupancy_grid_, sync_pt_pointcloud_);
    sync_->registerCallback(std::bind(&NodlNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ExactTime<OccupancyGrid, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void on_input_occupancy_grid_map(OccupancyGrid::ConstSharedPtr msg) override
  {
    sync_pt_occupancy_grid_.add(std::move(msg));
  }

  void on_input_pointcloud(PointCloud2::ConstSharedPtr msg) override
  {
    sync_pt_pointcloud_.add(std::move(msg));
  }

  void sync_callback(const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
  {
    auto result = filterPipeline(
      input_ogm,
      input_pc,
      tf2_,
      radius_search_2d_filter_,
      params_.base_link_frame,
      static_cast<int>(params_.cost_threshold));
    if (result) {
      pub_output_pointcloud_->publish(std::move(result));
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::optional<RadiusSearch2dFilter> radius_search_2d_filter_;

  message_filters::PassThrough<OccupancyGrid> sync_pt_occupancy_grid_;
  message_filters::PassThrough<PointCloud2> sync_pt_pointcloud_;
  std::shared_ptr<Sync> sync_;
};

}  // namespace occupancy_grid_map_outlier_filter

RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_map_outlier_filter::NodlNode)
