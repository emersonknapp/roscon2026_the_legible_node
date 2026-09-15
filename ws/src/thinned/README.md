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

## Unit testing

The tests in [test_outlier_filter.cpp](./test/test_outlier_filter.cpp) are quite simple on small data, but demonstrate the point that it is now possible to unit test the core functionality in a lightweight way without incurring the full overhead of ROS message passing and discovery.

Most ROS node tests with message passing end up flaky - even with intentional waits and padded timeouts.

The message-passing equivalent of _one_ of the tests in this suite is in [autoware_occupancy_grid_map_outlier_filter/test/test_pipeline_integration.cpp](../autoware_occupancy_grid_map_outlier_filter/test/test_pipeline_integration.cpp)

On my development computer I ran each test suite 1000 times in a row with `--gtest-repeat=1000`:

- the original ROS messaging-based test: ~4 seconds
- this direct-library-call test suite: ~0.04 seconds

That's 100 times faster - two orders of magnitude.
And the only thing we lost out on was testing that the node's interface is correct and wired up to the underlying library, a much thinner smoke test that can be done in its own dedicated step, rather than for the core business logic.

When applied across much larger suites and repeated runs, this speedup pays dividends.
Among other benefits, being able to run a test thousands of times helps shake out flakiness and track down nondeterministic issues like thread-safety bugs.
Not to mention that your developers are likely to run the full test suite locally more often, and that your CI bills will go down.
