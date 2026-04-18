# mujoco_sim

`mujoco_sim` is a standalone MuJoCo simulation repository.

It contains:

- `mujoco_core`: ROS-agnostic MuJoCo runtime and plant access layer
- `mujoco_viewer`: interactive MuJoCo viewer and standalone viewer executable
- `mujoco_models`: public MJCF models and assets used by the standalone examples

This repo is intentionally plain CMake.
It does not require `ament_cmake` or a ROS workspace to build.

## Scope

`mujoco_sim` is the simulation foundation layer.

It is responsible for:

- loading MuJoCo models
- owning `mjModel` / `mjData`
- stepping and resetting simulation
- actuator command application
- tracked-frame and force/torque sensor extraction
- standalone visualization
- packaging public example models

It is not responsible for:

- ROS topics or nodes
- `ros2_control`
- controller manager integration
- launch files
- robot-specific private assets

Those belong in downstream repos that consume `mujoco_sim`.

## Build

This repo is set up to build with `pixi`.

### Dependencies

Minimal environment dependencies:

- `mujoco`
- `cmake`
- `ninja`
- `pkg-config`
- `glfw`
- `yaml-cpp`
- `opencv`
- `gtest`

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

## Installed CMake Package

`mujoco_sim` installs an exported CMake package config.

Downstream repos can consume it with:

```cmake
find_package(mujoco_sim CONFIG REQUIRED)
```

Available exported targets include:

- `mujoco_sim::mujoco_core`
- `mujoco_sim::mujoco_viewer`

## Repository Layout

```text
mujoco_core/
mujoco_viewer/
mujoco_models/
cmake/
```

## Notes

- MuJoCo is resolved from `MUJOCO_PATH` or `CONDA_PREFIX`
- installed binaries carry runtime paths into the local install tree and pixi environment
- public example configs are installed under:
  - `share/mujoco_core/config`
- public model assets are installed under:
  - `share/mujoco_models`
