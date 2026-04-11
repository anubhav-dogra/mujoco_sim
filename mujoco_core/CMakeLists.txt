cmake_minimum_required(VERSION 3.10)
project(mujoco_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
include(CTest)

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

add_library(mujoco_core SHARED
  src/MujocoSimCore.cpp
)

target_include_directories(mujoco_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MUJOCO_PATH}/include/mujoco>
  $<BUILD_INTERFACE:${MUJOCO_PATH}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_directories(mujoco_core PUBLIC ${MUJOCO_PATH}/lib)
target_link_libraries(mujoco_core PUBLIC mujoco)

install(
  TARGETS mujoco_core
  EXPORT export_mujoco_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES include/mujoco_core/MujocoSimCore.h
  DESTINATION include/mujoco_core
)

install(
  DIRECTORY config/standalone
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_mujoco_sim_core
    test/test_mujoco_sim_core.cpp
    src/MujocoSimCore.cpp
  )

  target_include_directories(test_mujoco_sim_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_PATH}/include/mujoco
    ${MUJOCO_PATH}/include
  )

  target_compile_definitions(test_mujoco_sim_core PRIVATE
    MUJOCO_ROS2_INTERFACE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_directories(test_mujoco_sim_core PRIVATE ${MUJOCO_PATH}/lib)
  target_link_libraries(test_mujoco_sim_core
    mujoco
    GTest::gtest
    GTest::gtest_main
  )

  add_test(NAME test_mujoco_sim_core COMMAND test_mujoco_sim_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(mujoco_core)
ament_export_targets(export_mujoco_core HAS_LIBRARY_TARGET)

ament_package()
