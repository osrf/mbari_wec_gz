find_package(FreeSurfaceHydrodynamics REQUIRED)
set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 COMPONENTS system iostreams filesystem)

gz_add_plugin(IncidentWaves
  SOURCES
    IncidentWaves.cpp
  PUBLIC_LINK_LIBS
    FreeSurfaceHydrodynamics
  INCLUDE_DIRS
    ..
    ../..
)
