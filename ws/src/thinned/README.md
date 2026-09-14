# "Thin Node" Refactor

This package demonstrates a thinning refactor to [autoware_occupancy_grid_map_outlier_filter](../autoware_occupancy_grid_map_outlier_filter/).

1. ROS interface clearly declared (parameters, topics, and extensions)
1. Business logic is pulled into a separate standalone library

There is some good restructuring and code cleanup that could be done here, but we have intentionally left everything as close as possible to the way it was, while separating the "node from the code".

## Implementation Library

All business logic is pulled out of the node, and placed into a standalone C++ library target.

- API declaration: [outlier_filter.hpp](./include/thin_ogmof/outlier_filter.hpp)
- Implementation: [outlier_filter.cpp](./src/outlier_filter.cpp)

This is exported as library target `thin_ogmof::thin_ogmof`, which could be utilized by any downstream package wanting to run the same algorithm within their own data flow.

## Node

The node itself ([node.hpp](./include/thin_ogmof/node.hpp) and [node.cpp](./src/node.cpp)) is now extremely simple.

1. Construct the ROS interface: Declare parameters, create subscriptions and publisher
1. Connect the subscription outputs to a `message_filters::Synchronizer`
1. Pass necessary interfaces to some node "extensions"
   - `tf2_ros::TfListener` subscribes to `/tf` and `/tf_static` on our behalf
   - the internal `RadiusSearch2dFilterNodeMixin` declares some extra parameters
1. Dispatch all callbacks to the implementation library.

The node implementation, outside the constructor, is in its entirety:

```cpp
void Node::sync_callback(const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  auto result = filterPipeline(
    input_ogm,
    input_pc,
    tf2_,
    radius_search_2d_filter_,
    params_.base_link_frame,
    params_.cost_threshold);
  if (result) {
    pub_pointcloud_->publish(std::move(result));
  }
}
```

`node.cpp` is down from 475 lines to 76. Now that's pretty thin!

And more importantly, all that extra 400 lines of implementation is now possible to unit test and reuse in other contexts.
