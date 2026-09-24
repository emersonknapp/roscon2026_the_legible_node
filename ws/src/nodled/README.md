# NoDL Refactor

This package demonstrates the next step after the [thin-node refactor](../thinned/): describe the node's ROS interface declaratively and generate its routine plumbing.

`nodl_ogmof` reuses the filtering algorithm exported by `thin_ogmof`. It does not contain another copy of that business logic. Instead, it shows how [NoDL](https://github.com/ros/nodl) can become the source of truth for the node's parameters, publishers, subscriptions, documentation, and interface tests.

## Where to Look

| Path | Purpose |
| --- | --- |
| [`nodl/node.nodl.yaml`](./nodl/node.nodl.yaml) | Main node definition: parameters, topics, types, QoS, descriptions, and validation rules. Start here. |
| [`nodl/radius_search_2d_filter.nodl.yaml`](./nodl/radius_search_2d_filter.nodl.yaml) | Reusable description of the radius-search filter parameters. |
| [`nodl/tf_listener.nodl.yaml`](./nodl/tf_listener.nodl.yaml) | Documents the `/tf` and `/tf_static` subscriptions owned by `tf2_ros::TransformListener`. |
| [`src/node.cpp`](./src/node.cpp) | Handwritten behavior connecting the generated interface to synchronization, TF, and the filtering library. |
| [`CMakeLists.txt`](./CMakeLists.txt) | NoDL registration, C++ generation, component construction, and the interface conformance test. |
| [`doc/overview.md`](./doc/overview.md) | Sphinx page that renders the complete effective interface directly from the NoDL document. |

## The NoDL Documents

The main definition includes three documents:

```yaml
include:
  - ref: nodl://nodl_common_interfaces/node
  - ref: local://radius_search_2d_filter.nodl.yaml
  - ref: local://tf_listener.nodl.yaml
```

Together they describe the node's complete public interface:

- the standard ROS node infrastructure;
- this node's parameters, publisher, and input subscriptions;
- the radius-search filter's parameters; and
- the TF subscriptions created on the node's behalf by `tf2_ros::TransformListener`.

Splitting these concerns into included documents makes extension-owned interfaces visible without pretending that the node creates every endpoint itself. For example, `tf_listener.nodl.yaml` uses the `NO_GENERATE` role: NoDL includes its subscriptions in documentation and conformance checks, but leaves their construction to `tf2_ros`.

## Generated Interface

The following CMake call generates an abstract C++ base class from the main document:

```cmake
nodl_generate_cpp(
  occupancy_grid_map_outlier_filter_base
  nodl/node.nodl.yaml
)
```

The generated base class owns the mechanical parts of the ROS interface:

- parameter declaration and typed storage;
- defaults, descriptions, and validation;
- publisher creation with the declared QoS;
- subscription creation with the declared QoS; and
- abstract callbacks for each generated subscription.

Generated files are build artifacts under `build/nodl_ogmof/nodl_generated/`; they are not checked into this package.

## Handwritten Node

[`NodlNode`](./src/node.cpp) inherits from the generated `OccupancyGridMapOutlierFilterBase`. Its handwritten responsibilities are limited to behavior and integration:

1. Construct the TF buffer and listener.
2. Construct the optional radius-search filter from generated parameters.
3. Connect the two generated subscription callbacks to an exact-time synchronizer.
4. Pass synchronized messages and dependencies to `filterPipeline()` from `thin_ogmof`.
5. Publish the returned pointcloud through the generated publisher.

The generated subscription callbacks make that boundary explicit:

```cpp
void on_input_occupancy_grid_map(OccupancyGrid::ConstSharedPtr msg) override
{
  sync_pt_occupancy_grid_.add(std::move(msg));
}

void on_input_pointcloud(PointCloud2::ConstSharedPtr msg) override
{
  sync_pt_pointcloud_.add(std::move(msg));
}
```

The filtering callback remains ordinary application code:

```cpp
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
```

NoDL removes repetitive interface wiring; it does not hide the node's control flow or move business logic into generated code.

## Registration, Documentation, and Testing

`ament_nodl_register()` publishes the node definition and its included documents to the ament index. This makes the installed interface available to downstream NoDL tooling.

The same definition also drives two checks against interface drift:

- `nodl_docgen` renders the merged node interface in the package documentation, so topic and parameter tables do not need to be copied by hand.
- `nodl_add_conformance_test()` launches the component and verifies that its observed runtime interface matches the NoDL declaration.

The thin-node unit tests remain responsible for the filtering algorithm. The NoDL conformance test covers the much smaller question left at the ROS boundary: whether the declared interface is actually present and correctly wired at runtime.
