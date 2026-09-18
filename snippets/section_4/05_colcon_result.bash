$ colcon test --packages-select thin_ogmof
Starting >>> thin_ogmof
Finished <<< thin_ogmof [4.58s]  [ with test failures ]

Summary: 1 package finished [4.72s]
  1 package had test failures: thin_ogmof

$ colcon test-result --verbose
...
AssertionError: NoDL conformance failed for '/thin_ogmof': 
      [missing] publishers '/thin_ogmof/output/pointcloud': expected type 'PointCloud2' was not observed
      [missing] subscriptions '/thin_ogmof/input/pointcloud': expected type 'PointCloud2' was not observed
