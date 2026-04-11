cmake_minimum_required(VERSION 3.10)
project(mujoco_models)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
