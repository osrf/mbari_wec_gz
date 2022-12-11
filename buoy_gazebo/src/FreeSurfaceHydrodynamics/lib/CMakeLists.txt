cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(WaveBodyInteractions)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(ament_index_cpp)

set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71.0 COMPONENTS system iostreams filesystem)

add_library(FreeSurfaceHydrodynamics SHARED interp1d.cpp LinearIncidentWave.cpp FS_Hydrodynamics.cpp)
#target_link_libraries(FreeSurfaceHydrodynamics PUBLIC Eigen3::Eigen Boost::system Boost::iostreams Boost::filesystem ament_index_cpp::ament_index_cpp)
target_link_libraries(FreeSurfaceHydrodynamics PUBLIC Eigen3::Eigen Boost::system Boost::iostreams Boost::filesystem)
set_property(TARGET FreeSurfaceHydrodynamics PROPERTY CXX_STANDARD 17)

#set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
add_executable(Test_IncidentWave  Test_IncidentWave.cpp )
target_link_libraries(Test_IncidentWave PUBLIC FreeSurfaceHydrodynamics)
set_property(TARGET Test_IncidentWave PROPERTY CXX_STANDARD 17)

add_executable(Test_FS_Hydrodynamics Test_FS_Hydrodynamics.cpp)
target_link_libraries(Test_FS_Hydrodynamics PUBLIC FreeSurfaceHydrodynamics)
set_property(TARGET Test_FS_Hydrodynamics PROPERTY CXX_STANDARD 17)
