This repository contains packages used for Gazebo simulation of marine buoys.

Start from [MBARI WEC](https://github.com/osrf/mbari_wec).

## Development

### Profiler

By default, the profiler isn't enabled. To enable it at compilation time,
pass the `-DENABLE_PROFILER=1` CMake argument. For example, to compile tests
and enable the profiler with `colcon`:

```
colcon build --cmake-args ' -DBUILD_TESTING=false' ' -DENABLE_PROFILER=1'
```

See more on [this tutorial](https://gazebosim.org/api/common/4.4/profiler.html).
