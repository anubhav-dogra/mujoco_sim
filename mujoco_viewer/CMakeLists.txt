cmake_minimum_required(VERSION 3.10)
project(mujoco_viewer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mujoco_core REQUIRED)
find_package(Threads REQUIRED)

if(DEFINED ENV{MUJOCO_PATH} AND NOT "$ENV{MUJOCO_PATH}" STREQUAL "")
  set(MUJOCO_PATH "$ENV{MUJOCO_PATH}")
elseif(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  set(MUJOCO_PATH "$ENV{CONDA_PREFIX}")
endif()

if(NOT EXISTS "${MUJOCO_PATH}/include/mujoco/mujoco.h")
  message(FATAL_ERROR
    "MuJoCo headers not found at ${MUJOCO_PATH}/include/mujoco/mujoco.h. "
    "Set MUJOCO_PATH to your MuJoCo install or build inside pixi.")
endif()

add_library(mujoco_viewer SHARED
  src/MujocoViewer.cpp
  src/ViewerControls.cpp
)

target_include_directories(mujoco_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MUJOCO_PATH}/include/mujoco>
  $<BUILD_INTERFACE:${MUJOCO_PATH}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_directories(mujoco_viewer PUBLIC ${MUJOCO_PATH}/lib)
target_link_libraries(mujoco_viewer
  mujoco_core::mujoco_core
  mujoco
  glfw
  Threads::Threads
)

ament_target_dependencies(mujoco_viewer
  glfw3
  OpenCV
)

add_executable(mujoco_sim_viewer
  src/mujoco_sim_viewer.cpp
)

target_include_directories(mujoco_sim_viewer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MUJOCO_PATH}/include/mujoco
  ${MUJOCO_PATH}/include
)

target_link_directories(mujoco_sim_viewer PRIVATE ${MUJOCO_PATH}/lib)
target_link_libraries(mujoco_sim_viewer
  mujoco_viewer
  mujoco_core::mujoco_core
  mujoco
  glfw
  yaml-cpp
  Threads::Threads
)

ament_target_dependencies(mujoco_sim_viewer
  glfw3
  OpenCV
)

install(
  TARGETS mujoco_viewer
  EXPORT export_mujoco_viewer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS mujoco_sim_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    include/mujoco_viewer/MujocoViewer.h
    include/mujoco_viewer/ViewerControls.h
  DESTINATION include/mujoco_viewer
)

ament_export_include_directories(include)
ament_export_libraries(mujoco_viewer)
ament_export_targets(export_mujoco_viewer HAS_LIBRARY_TARGET)
ament_export_dependencies(mujoco_core glfw3 OpenCV yaml-cpp)

ament_package()
