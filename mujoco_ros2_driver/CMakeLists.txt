cmake_minimum_required(VERSION 3.10)
project(mujoco_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_core REQUIRED)
find_package(mujoco_viewer REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(${PROJECT_NAME} SHARED src/mujoco_ros2_driver.cpp)

ament_target_dependencies(${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
target_link_libraries(${PROJECT_NAME}
  mujoco_core::mujoco_core
  mujoco_viewer::mujoco_viewer
)
target_include_directories(mujoco_ros2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mujoco_ros2_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

pluginlib_export_plugin_description_file(hardware_interface mujoco_ros2_driver.xml)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mujoco_ros2_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  mujoco_core
  mujoco_viewer
)

ament_package()
