find_package(FreeSurfaceHydrodynamics 1.4.2 REQUIRED)

# Set policy CMP0167 so we avoid warnings about deprecated FindBoost
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.83.0 REQUIRED CONFIG COMPONENTS system iostreams filesystem)

gz_add_plugin(IncidentWaves
  SOURCES
    IncidentWaves.cpp
  PUBLIC_LINK_LIBS
    FreeSurfaceHydrodynamics
  INCLUDE_DIRS
    ..
    ../..
)
