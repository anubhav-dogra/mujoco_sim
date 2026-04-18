add_library(mujoco_viewer SHARED
  src/MujocoViewer.cpp
  src/ViewerControls.cpp
)
add_library(mujoco_sim::mujoco_viewer ALIAS mujoco_viewer)

target_include_directories(mujoco_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mujoco_viewer PUBLIC
  mujoco_core
  mujoco::mujoco
  glfw
  ${OpenCV_LIBS}
)

add_executable(mujoco_sim_viewer
  src/mujoco_sim_viewer.cpp
)

target_include_directories(mujoco_sim_viewer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mujoco_sim_viewer
  mujoco_viewer
  mujoco_core
  mujoco::mujoco
  yaml-cpp::yaml-cpp
  ${OpenCV_LIBS}
)

install(
  TARGETS mujoco_viewer
  EXPORT mujoco_simTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS mujoco_sim_viewer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  FILES
    include/mujoco_viewer/MujocoViewer.h
    include/mujoco_viewer/ViewerControls.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_viewer
)
