gz_add_plugin(ElectroHydraulicPTO
  SOURCES
    ElectroHydraulicPTO.cpp
  INCLUDE_DIRS
    ..
  EXTRA_ROS_PKGS
    splinter_ros
)

add_library(splinter-3-0 SHARED IMPORTED)
set_target_properties(splinter-3-0 PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/src/ElectroHydraulicPTO/lib/libsplinter-3-0.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/src/ElectroHydraulicPTO/include/splinter"
)
target_link_libraries(ElectroHydraulicPTO PUBLIC splinter-3-0)

install(FILES lib/libsplinter-3-0.so
  DESTINATION lib)
