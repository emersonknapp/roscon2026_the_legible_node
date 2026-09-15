namespace outlier_filter
{

/// @brief Size output to match input's layout and point count, ready to be filled.
void initializePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief Copy input's metadata onto output and recompute its width/row_step from data size.
void finalizePointCloud2(const PointCloud2 & input, PointCloud2 & output);

/// @brief Append input's points onto the end of output.
void concatPointCloud2(PointCloud2 & output, const PointCloud2 & input);

/// @brief Partition input_pc into points ahead of (front_pc) and behind (behind_pc) the vehicle.
void splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc,
  PointCloud2 & front_pc,
  PointCloud2 & behind_pc);

/// @brief Bucket each point by its occupancy grid cost into high/low confidence and out-of-map clouds.
void filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map,
  const PointCloud2 & pointcloud,
  const int cost_threshold,
  PointCloud2 & high_confidence,
  PointCloud2 & low_confidence,
  PointCloud2 & out_ogm);

/// @brief Full outlier-filter pipeline: split, transform, classify, radius-filter, recombine.
/// @param radius_search Optional radius filter; when absent, low-confidence points are dropped.
/// @return Filtered cloud in base_link_frame, or nullptr if a transform fails.
std::unique_ptr<PointCloud2> filterPipeline(
  const OccupancyGrid::ConstSharedPtr & input_ogm,
  const PointCloud2::ConstSharedPtr & input_pc,
  const std::shared_ptr<tf2_ros::Buffer> tf2,
  std::optional<RadiusSearch2dFilter> radius_search,
  const std::string & base_link_frame,
  const int cost_threshold);

}  // namespace outlier_filter
