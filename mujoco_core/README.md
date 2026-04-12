# mujoco_core

`mujoco_core` contains the MuJoCo plant backend for this workspace.

It is intentionally ROS-free. Its job is to:

- load an MJCF model
- own `mjModel` and `mjData`
- step and reset the simulation
- apply actuator commands
- expose joint state, tracked-frame, and FT-sensor reads
- expose name-based lookup for MuJoCo state joints and control targets

## Responsibility

`MujocoSimCore` is the central class.

It should know about:

- MuJoCo model loading
- MuJoCo stepping
- MuJoCo state access
- MuJoCo control targets
- MuJoCo sensor extraction

It should not know about:

- ROS topics
- `ros2_control`
- controller-manager semantics
- robot-specific runtime policy

## Two Namespaces

The important concept in `mujoco_core` is that state and control are not always the same namespace.

State is joint-based:

- exported joint names
- `qpos` indices
- `qvel` indices

Control is actuator-target-based:

- actuator/control target names
- actuator indices used for `ctrl` and `actuator_force`

For simple arm joints, these often line up naturally.

For tendon-driven mechanisms, such as a coupled gripper:

- state may come from a MuJoCo joint
- control may go to a tendon-driven actuator with a different name

That is why the core exposes both:

- `joint_state_indices_by_name()`
- `control_indices_by_name()`

## Current Public API Shape

The main APIs used by higher layers are:

- `model()`
- `data()`
- `state_mutex()`
- `step()`
- `reset()`
- `set_position_command()`
- `set_effort_command()`
- `joint_names()`
- `joint_state_indices_by_name()`
- `joint_position_indices()`
- `joint_velocity_indices()`
- `control_indices_by_name()`
- `ft_sensor_configs()`
- `force_torque_states()`
- `tracked_frame_states()`

## Plant Versus Runtime Policy

Keep this split strict:

- plant properties belong in MJCF
  - friction
  - damping
  - masses
  - actuator wiring
- runtime policy belongs outside the core
  - ROS-facing interface selection
  - compensation layers
  - controller wiring
  - launch behavior

The core may accept generic runtime flags such as:

- control mode
- gravity compensation enable
- damping gain
- simulation frequency

But it should not become a robot-specific behavior layer.

## Intended Consumers

`mujoco_core` is shared by:

- standalone viewer path
- `mujoco_ros2_driver`
- future non-ROS tools such as IK, RL, or system identification helpers

That is why the API is framed around MuJoCo concepts rather than ROS concepts.
