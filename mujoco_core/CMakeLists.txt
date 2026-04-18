include(CTest)

add_library(mujoco_core SHARED
  src/MujocoSimCore.cpp
)
add_library(mujoco_sim::mujoco_core ALIAS mujoco_core)

target_include_directories(mujoco_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mujoco_core PUBLIC mujoco::mujoco)

install(
  TARGETS mujoco_core
  EXPORT mujoco_simTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  FILES include/mujoco_core/MujocoSimCore.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mujoco_core
)

install(
  DIRECTORY config/standalone
  DESTINATION ${CMAKE_INSTALL_DATADIR}/mujoco_core/config
)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_mujoco_sim_core
    test/test_mujoco_sim_core.cpp
  )

  target_compile_definitions(test_mujoco_sim_core PRIVATE
    MUJOCO_ROS2_INTERFACE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(test_mujoco_sim_core
    mujoco_core
    GTest::gtest
    GTest::gtest_main
  )

  add_test(NAME test_mujoco_sim_core COMMAND test_mujoco_sim_core)
endif()
