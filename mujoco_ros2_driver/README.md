# mujoco_ros2_driver

`mujoco_ros2_driver` is the `ros2_control` hardware plugin for this workspace.

It adapts:

- ROS joint and sensor interfaces

to:

- `MujocoSimCore`

## Responsibility

The driver should do three things well:

1. parse `HardwareInfo` from the URDF `ros2_control` block
2. bind ROS interfaces to MuJoCo state joints and MuJoCo control targets
3. run the MuJoCo simulation lifecycle inside a `SystemInterface`

It should not:

- duplicate plant logic from `mujoco_core`
- hardcode robot-specific behavior
- hide controller semantics inside the core

## Key Concept: Binding

The important design choice is that the driver does not assume:

- ROS joint name == MuJoCo state joint name == MuJoCo control target name

Instead it resolves a binding per exported ROS joint.

Current binding fields are:

- `ros_joint_name`
- `mujoco_state_joint_name`
- `mujoco_control_name`
- `qpos_index`
- `qvel_index`
- `control_index`

This matters because:

- normal arm joints are usually direct joint-actuated
- some mechanisms, such as tendon-driven grippers, are not

For those mechanisms:

- state can still come from a joint
- command can go to a differently named actuator target

That is handled generically by the binding layer.

## Where Metadata Belongs

Default behavior:

- `mujoco_state_joint_name = ros_joint_name`
- `mujoco_control_name = ros_joint_name`

Only override when needed.

Today, override metadata comes from the URDF `ros2_control` block, for example:

- `control_name`
- optionally `state_joint_name`

This is interface-binding metadata, not plant metadata.

It belongs in the driver-facing description layer, not in controller code.

## Read And Write Model

`read()`:

- uses the binding's `qpos_index` and `qvel_index`
- uses the binding's `control_index` for effort readback
- fills ROS joint state interfaces
- fills FT sensor state interfaces

`write()`:

- reads ROS command interfaces
- uses the binding's `control_index`
- forwards position or effort commands into `MujocoSimCore`

## Threads

The driver owns the simulation runtime:

- a simulation thread steps MuJoCo
- an optional viewer thread renders the same core

`read()` is a state snapshot.

`write()` is a command handoff.

Physics stepping should not depend on `read()` timing.

## Configuration Surface

The intended hardware params are narrow:

- `xml_location`
- `control_mode`
- `simulation_frequency`
- `visualization_enabled`

FT sensors are configured from the URDF sensor entries:

- `frame_id`
- `force_sensor_name`
- `torque_sensor_name`

Additional params should only be added when they represent real runtime behavior that belongs at the adapter layer.

## Design Rule Going Forward

When a mechanism needs special wiring, do not add robot-specific branches like:

- `if robotiq ...`

Instead:

- expose the needed MuJoCo names in the description metadata
- let the generic binding layer resolve them

That keeps the driver reusable across:

- direct joint-actuated robots
- tendon-driven mechanisms
- future mixed actuation layouts
