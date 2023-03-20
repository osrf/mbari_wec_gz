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

gz_add_plugin(IncidentWaves
  SOURCES
  IncidentWaves.cpp
  INCLUDE_DIRS
    ..
)

target_link_libraries(IncidentWaves PUBLIC FreeSurfaceHydrodynamics)

