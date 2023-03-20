find_package(ament_index_cpp REQUIRED)

set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 COMPONENTS system iostreams filesystem)

include(FetchContent)
FetchContent_Declare(FreeSurfaceHydrodynamics      # name 
  GIT_REPOSITORY https://github.com/hamilton8415/FreeSurfaceHydrodynamics.git  # repository
  GIT_TAG        v1.2.3  # tag
)
FetchContent_MakeAvailable(FreeSurfaceHydrodynamics)



gz_add_plugin(WaveBodyInteractions
  SOURCES
    WaveBodyInteractions.cpp
  INCLUDE_DIRS
    ..
    ../..
  EXTRA_ROS_PKGS
    simple_interp
)

target_link_libraries(WaveBodyInteractions PUBLIC FreeSurfaceHydrodynamics ament_index_cpp::ament_index_cpp)
