// pared down a bit from autoware / node.hpp

class OccupancyGridMapOutlierFilterComponent : public rclcpp::Node
{
public:
  explicit OccupancyGridMapOutlierFilterComponent(const rclcpp::NodeOptions & options);

private:
  void onOccupancyGridMapAndPointCloud2(
    const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc);

  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  message_filters::Subscriber<OccupancyGrid> occupancy_grid_map_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_ptr_;
  ...
