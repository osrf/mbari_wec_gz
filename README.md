This repository contains packages used for Gazebo simulation of marine buoys.

Start from [Buoy entrypoint](https://github.com/osrf/buoy_entrypoint).

## Development

### Profiler

By default, the profiler isn't enabled. To enable it at compilation time,
pass the `-DENABLE_PROFILER=1` CMake argument. For example, to compile tests
and enable the profiler with `colcon`:

```
colcon build --cmake-args ' -DBUILD_TESTING=false' ' -DENABLE_PROFILER=1'
```

See more on [this tutorial](https://ignitionrobotics.org/api/common/4.4/profiler.html).
