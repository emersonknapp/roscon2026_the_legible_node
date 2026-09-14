#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/search/kdtree.h"


#include <optional>
#include <string>

namespace occupancy_grid_map_outlier_filter
{

bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame,
  sensor_msgs::msg::PointCloud2 & output);

geometry_msgs::msg::PoseStamped getPoseStamped(
  const tf2_ros::Buffer & tf2,
  const std::string & target_frame_id,
  const std::string & src_frame_id,
  const rclcpp::Time & time);

std::optional<char> getCost(const nav_msgs::msg::OccupancyGrid & map, const double & x, const double & y);

class RadiusSearch2dFilter
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Pose = geometry_msgs::msg::Pose;

  struct Config
  {
    float search_radius;
    float min_points_and_distance_ratio;
    int min_points;
    int max_points;
    uint32_t max_filter_points_nb;
  };

  explicit RadiusSearch2dFilter(Config config);
  void filter(const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier);
  void filter(
    const PointCloud2 & high_conf_xyz_cloud,
    const PointCloud2 & low_conf_xyz_cloud,
    const Pose & pose,
    PointCloud2 & output,
    PointCloud2 & outlier);

private:
  Config config_;
  pcl::search::Search<pcl::PointXY>::Ptr kd_tree_;
};

class RadiusSearch2dFilterNodeMixin
{
  explicit RadiusSearch2dFilterNodeMixin(rclcpp::node_interfaces::NodeParametersInterface & node_params);
  RadiusSearch2dFilter::Config config;
};



}  // namespace occupancy_grid_map_outlier_filter
