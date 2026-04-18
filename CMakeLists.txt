cmake_minimum_required(VERSION 3.16)
project(mujoco_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(DEFINED ENV{MUJOCO_PATH} AND NOT "$ENV{MUJOCO_PATH}" STREQUAL "")
  set(MUJOCO_PATH "$ENV{MUJOCO_PATH}")
elseif(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  set(MUJOCO_PATH "$ENV{CONDA_PREFIX}")
endif()

if(NOT DEFINED MUJOCO_PATH OR MUJOCO_PATH STREQUAL "")
  message(FATAL_ERROR
    "MuJoCo install path was not found. Set MUJOCO_PATH or build inside pixi so CONDA_PREFIX is available.")
endif()

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS "${MUJOCO_PATH}/include"
  NO_DEFAULT_PATH
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco
  PATHS "${MUJOCO_PATH}/lib"
  NO_DEFAULT_PATH
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo headers or library were not found under ${MUJOCO_PATH}. "
    "Expected ${MUJOCO_PATH}/include/mujoco/mujoco.h and ${MUJOCO_PATH}/lib/libmujoco.*")
endif()

list(APPEND CMAKE_INSTALL_RPATH "${MUJOCO_PATH}/lib")

add_library(mujoco::mujoco UNKNOWN IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(mujoco_core)
add_subdirectory(mujoco_viewer)
add_subdirectory(mujoco_models)

configure_package_config_file(
  cmake/mujoco_simConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_sim"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(EXPORT mujoco_simTargets
  FILE mujoco_simTargets.cmake
  NAMESPACE mujoco_sim::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_sim"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_simConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/mujoco_sim"
)
