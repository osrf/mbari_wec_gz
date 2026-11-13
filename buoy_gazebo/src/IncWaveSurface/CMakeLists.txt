find_package(FreeSurfaceHydrodynamics 1.4.2 REQUIRED)

# Set policy CMP0167 so we avoid warnings about deprecated FindBoost
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
set(BOOST_USE_STATIC_LIBS OFF)
set(BOOST_USE_MULTITHREADED ON)
set(BOOST_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.83.0 REQUIRED CONFIG COMPONENTS system iostreams filesystem)

# Server side: serves the wave components
gz_add_plugin(IncWaveSurface
  SOURCES
    IncWaveSurface.cpp
  PUBLIC_LINK_LIBS
    FreeSurfaceHydrodynamics
  INCLUDE_DIRS
    ..
)

# GUI side: draws the surface as an Ogre-next dynamic mesh
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED COMPONENTS ogre2)
find_package(gz_ogre_next_vendor REQUIRED)  # puts OGRE-Next on PKG_CONFIG_PATH
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_NEXT REQUIRED IMPORTED_TARGET OGRE-Next OGRE-Next-Hlms OGRE-Next-Overlay)
pkg_get_variable(OGRE_NEXT_INCLUDE_ROOT OGRE-Next includedir)
find_package(Threads REQUIRED)

gz_add_plugin(IncWaveSurfaceVisual
  SOURCES
    IncWaveSurfaceVisual.cpp
    WaveSurfaceField.cpp
  PUBLIC_LINK_LIBS
    gz-rendering::gz-rendering
    gz-rendering::ogre2
    PkgConfig::OGRE_NEXT
    Threads::Threads
  INCLUDE_DIRS
    ..
    ${OGRE_NEXT_INCLUDE_ROOT}/OGRE-Next/Hlms/Common
    ${OGRE_NEXT_INCLUDE_ROOT}/OGRE-Next/Hlms/Pbs
)

# Reflect gz-rendering's sky by default
get_filename_component(GZ_RENDERING_PREFIX "${gz-rendering8_DIR}/../../.." ABSOLUTE)
set(INC_WAVE_SURFACE_SKY
  "${GZ_RENDERING_PREFIX}/share/gz/gz-rendering8/ogre2/media/materials/textures/skybox.dds")
# Ogre otherwise warns that it can't tell whether this is a debug build
target_compile_definitions(IncWaveSurfaceVisual PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)

if(EXISTS "${INC_WAVE_SURFACE_SKY}")
  target_compile_definitions(IncWaveSurfaceVisual PRIVATE
    INC_WAVE_SURFACE_DEFAULT_ENV_MAP="${INC_WAVE_SURFACE_SKY}")
endif()

# The surface is computed every frame, so optimize it even when the build
# type is unset
if(NOT CMAKE_BUILD_TYPE)
  target_compile_options(IncWaveSurfaceVisual PRIVATE -O2)
endif()
