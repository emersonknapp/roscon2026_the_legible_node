// from autoware / node.cpp - demo being deep into a node file, and it's just "onMessage", "everything is a callback"

void OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2(
  const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  // Transform to occupancy grid map frame

  PointCloud2 input_behind_pc{};
  PointCloud2 input_front_pc{};
  initializerPointCloud2(*input_pc, input_front_pc);
  initializerPointCloud2(*input_pc, input_behind_pc);
  // Split pointcloud into front and behind of the vehicle to reduce the calculation cost
  splitPointCloudFrontBack(input_pc, input_front_pc, input_behind_pc);
  finalizePointCloud2(*input_pc, input_front_pc);
