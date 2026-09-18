class NodlNode(OccupancyGridMapOutlierFilterBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # no setup needed!

        # access parameters on params_
        map_frame = self.params_.map_frame

        # publishers already created
        self.pub_output_pointcloud.publish(PointCloud2())

    def on_input_occupancy_grid_map(self, msg):
        # implement callback
        ...
