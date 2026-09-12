# Minimise Dependencies Report

Goal: make this node buildable/runnable with minimal external dependencies, while
keeping the fat callbacks intact to demonstrate the "thin node" concept.

## Removed

| Dependency | Why it was safe to remove |
|---|---|
| `autoware_pointcloud_preprocessor` | Dead `filter.hpp` include; the class only inherits `rclcpp::Node`. |
| `autoware_utils` | Only used for debug/timing scaffolding (`StopWatch`, `DebugPublisher`, `TimeKeeper`, `ScopedTimeTrack`) + one `transform2pose` call. Scaffolding deleted; `transform2pose` inlined. |
| `autoware_internal_debug_msgs` | Transitive via `autoware_utils`; gone once timing was removed. |
| `pcl_ros` | Only used for `pcl_ros::transformPointCloud`; replaced with a local `PointCloud2Iterator` + Eigen transform. |
| Boost | Only `boost::optional`/`boost::none`; replaced with `std::optional`/`std::nullopt`. |
| OpenCV | Never used. |
| OpenMP | Never used (no `#pragma omp`). |
| `autoware_cmake` / `autoware_lint_common` | Autoware build tooling; switched to plain `ament_cmake_auto`. |

## Kept (standard ROS / PCL only)

`rclcpp`, `rclcpp_components`, `sensor_msgs`, `nav_msgs`, `std_msgs`,
`message_filters`, `tf2_ros`, `tf2_eigen`, `pcl_conversions`, `libpcl-all-dev`, `Eigen3`.

## Behaviour

Filtering logic is unchanged. The large callbacks are deliberately left as-is so
they can later be extracted into a library to demonstrate the thin-node refactor.

The `publish_processing_time_detail` parameter was removed along with the timing code.
