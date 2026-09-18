class NodlNode final : public OccupancyGridMapOutlierFilterBase
{
public:
  explicit NodlNode(const rclcpp::NodeOptions & options)
  : OccupancyGridMapOutlierFilterBase(options)
  {
    // no setup needed!

    // access parameters on params_
    int map = params_.map_frame;

    // publishers already created
    pub_output_pointcloud_->publish(PointCloud2());
  }

private:
  void on_input_occupancy_grid_map(OccupancyGrid::ConstSharedPtr msg) override
  {
    ...
  }
}