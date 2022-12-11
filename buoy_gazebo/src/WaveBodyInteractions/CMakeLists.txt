find_package(ament_index_cpp REQUIRED)

add_subdirectory(lib)

gz_add_plugin(WaveBodyInteractions
  SOURCES
    WaveBodyInteractions.cpp
  INCLUDE_DIRS
    ..
    lib
  EXTRA_ROS_PKGS
    simple_interp
)

target_link_libraries(WaveBodyInteractions PUBLIC FreeSurfaceHydrodynamics ament_index_cpp::ament_index_cpp)

