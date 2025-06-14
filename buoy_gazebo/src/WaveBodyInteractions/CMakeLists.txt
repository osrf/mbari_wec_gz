find_package(ament_index_cpp REQUIRED)

find_package(FreeSurfaceHydrodynamics REQUIRED)

# Set policy CMP0167 so we avoid warnings about deprecated FindBoost
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.83.0 REQUIRED CONFIG COMPONENTS system iostreams filesystem)

gz_add_plugin(WaveBodyInteractions
  SOURCES
    WaveBodyInteractions.cpp
  PUBLIC_LINK_LIBS
    FreeSurfaceHydrodynamics
  INCLUDE_DIRS
    ..
    ../..
    ../LatentData
  EXTRA_ROS_PKGS
    simple_interp
    ament_index_cpp
)
