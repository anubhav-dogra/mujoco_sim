# mujoco_sim

`mujoco_sim` is a MuJoCo simulation and ROS 2 integration repository.

It contains:

- `mujoco_core`: MuJoCo runtime and plant access layer
- `mujoco_viewer`: interactive MuJoCo viewer and standalone viewer executable
- `mujoco_models`: public MJCF models and assets used by the standalone examples
- `mujoco_ros2_driver`: `ros2_control` hardware plugin for MuJoCo-backed simulation

This repo is structured as an `ament_cmake` / `colcon` workspace and builds through `pixi`.

## Scope

`mujoco_sim` is the public MuJoCo simulation stack.

It is responsible for:

- loading MuJoCo models
- owning `mjModel` / `mjData`
- stepping and resetting simulation
- actuator command application
- tracked-frame and force/torque sensor extraction
- standalone visualization
- packaging public example models
- exposing a ROS 2 hardware plugin for higher-level control integration

It is not responsible for:

- robot-specific private assets

Private robot descriptions, deployment assets, and private bringup remain in downstream/private repos.

## Build

This repo is set up to build with `pixi`.

### Dependencies

Minimal environment dependencies:

- `mujoco`
- `cmake`
- `ninja`
- `pkg-config`
- `colcon-common-extensions`
- `glfw`
- `yaml-cpp`
- `opencv`
- `gtest`
- `ros-humble-ament-cmake`
- `ros-humble-hardware-interface`
- `ros-humble-pluginlib`
- `ros-humble-rclcpp`
- `ros-humble-rclcpp-lifecycle`

### Build Commands

```bash
pixi run build
```

Clean and rebuild:

```bash
pixi run rebuild
```

Run tests:

```bash
pixi run test
```

## Run the Standalone Viewer

Pendulum example:

```bash
pixi run sim_viewer_pendulum
```

## Repository Layout

```text
mujoco_core/
mujoco_viewer/
mujoco_models/
mujoco_ros2_driver/
```

## Notes

- MuJoCo is resolved from `MUJOCO_PATH` or `CONDA_PREFIX`
- public example configs are installed under:
  - `share/mujoco_core/config`
- public model assets are installed under:
  - `share/mujoco_models`
