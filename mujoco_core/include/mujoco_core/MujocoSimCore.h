#pragma once

#include <mujoco/mujoco.h>
#include <cstddef>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <array>
#include <string>
#include <vector>

enum ControlMode { POSITION, VELOCITY, TORQUE, UNKNOWN };

/**
 * @brief MuJoCo-only simulation core.
 *
 * This class owns the MuJoCo model/data handles and exposes stepping, reset,
 * command application, and read-only state access for higher-level adapters.
 */
class MujocoSimCore {
   public:
    /**
     * @brief Static configuration used to construct the simulation core.
     */
    struct Config {
        std::string xml_location;
        std::string plugin_directory;
        std::vector<std::string> tracked_frame_names;
        std::vector<std::string> ft_force_sensor_names;
        std::vector<std::string> ft_torque_sensor_names;
        std::vector<std::string> ft_sensor_frame_ids;
        ControlMode control_mode = TORQUE;
        bool enable_gravity_compensation = false;
        bool visualization_enabled = true;
        double joint_damping_gain = 0.0;
        int simulation_frequency = 1000;
    };

    enum class TrackedFrameType { BODY, SITE };

    /**
     * @brief Force/torque sensor slice configuration.
     */
    struct ForceTorqueSensorInfo {
        std::string frame_id;
        std::string force_sensor_name;
        std::string torque_sensor_name;
        int force_sensor_id = -1;
        int torque_sensor_id = -1;
        int force_sensor_adr = -1;
        int torque_sensor_adr = -1;
    };

    /**
     * @brief Resolved tracked frame metadata.
     */
    struct TrackedFrameInfo {
        std::string name;
        TrackedFrameType type = TrackedFrameType::BODY;
        int object_id = -1;
    };

    /**
     * @brief Runtime pose for a tracked body or site.
     */
    struct TrackedFrameState {
        std::string name;
        std::array<double, 3> position{};
        std::array<double, 4> orientation{};
    };

    /**
     * @brief Runtime wrench data for a configured force/torque output.
     */
    struct ForceTorqueSensorWrench {
        std::string frame_id;
        bool available = false;
        std::array<double, 3> force{};
        std::array<double, 3> torque{};
    };

    /**
     * @brief Construct the simulation core from static configuration.
     * @param config MuJoCo model, control, and observation configuration.
     */
    explicit MujocoSimCore(const Config& config);
    MujocoSimCore(const MujocoSimCore&) = delete;
    MujocoSimCore& operator=(const MujocoSimCore&) = delete;
    MujocoSimCore(MujocoSimCore&&) = default;
    MujocoSimCore& operator=(MujocoSimCore&&) = default;

    /**
     * @brief Access the owned MuJoCo model.
     * @return Borrowed pointer to the active mjModel.
     */
    mjModel* model() const;

    /**
     * @brief Access the owned MuJoCo runtime data.
     * @return Borrowed pointer to the active mjData.
     */
    mjData* data() const;

    /**
     * @brief Access the mutex guarding MuJoCo state.
     * @return Recursive mutex protecting model/data mutation and reads.
     */
    std::recursive_mutex& state_mutex() const;

    /**
     * @brief Advance the simulation by one MuJoCo step.
     */
    void step();

    /**
     * @brief Reset the simulation to the initial state or first keyframe.
     */
    void reset();

    /**
     * @brief Report whether the loaded model exposes fixed cameras.
     * @return True if at least one MuJoCo camera exists in the model.
     */
    bool has_fixed_cameras() const;

    /**
     * @brief Set a position-style actuator command.
     * @param actuator_id MuJoCo actuator index.
     * @param value Position target written to ctrl.
     */
    void set_position_command(std::size_t actuator_id, double value);

    /**
     * @brief Set an effort-style actuator command.
     * @param actuator_id MuJoCo actuator index.
     * @param value Effort target written to the internal command buffer.
     */
    void set_effort_command(std::size_t actuator_id, double value);

    /**
     * @brief Get the exported joint names.
     * @return Read-only list of joint names.
     */
    const std::vector<std::string>& joint_names() const;

    /**
     * @brief Get the exported joint-state index for each joint name.
     * @return Read-only map from joint name to exported joint-state slot.
     */
    const std::unordered_map<std::string, std::size_t>& joint_state_indices_by_name() const;

    /**
     * @brief Get qpos indices for exported joints.
     * @return Read-only list of MuJoCo qpos indices.
     */
    const std::vector<std::size_t>& joint_position_indices() const;

    /**
     * @brief Get qvel indices for exported joints.
     * @return Read-only list of MuJoCo qvel indices.
     */
    const std::vector<std::size_t>& joint_velocity_indices() const;

    /**
     * @brief Get effort/control indices for exported joints.
     * @return Read-only list of actuator indices aligned with actuator order, not joint-state order.
     */
    const std::vector<std::size_t>& joint_effort_indices() const;

    /**
     * @brief Get the control-target name to actuator index mapping.
     * @return Read-only name-to-actuator map for actuator names and direct joint-driven actuator aliases.
     */
    const std::unordered_map<std::string, std::size_t>& control_indices_by_name() const;

    /**
     * @brief Get configured force/torque sensor metadata.
     * @return Read-only list of configured FT outputs.
     */
    const std::vector<ForceTorqueSensorInfo>& ft_sensor_configs() const;

    /**
     * @brief Get configured tracked frame metadata.
     * @return Read-only list of resolved tracked frames.
     */
    const std::vector<TrackedFrameInfo>& tracked_frame_configs() const;

    /**
     * @brief Read current wrench values from MuJoCo sensor data.
     * @return Snapshot of force/torque outputs.
     */
    std::vector<ForceTorqueSensorWrench> force_torque_states() const;
    std::vector<ForceTorqueSensorWrench> force_torque_states(const mjData& data) const;

    /**
     * @brief Read current poses for all configured tracked frames.
     * @return Snapshot of tracked-frame poses.
     */
    std::vector<TrackedFrameState> tracked_frame_states() const;
    std::vector<TrackedFrameState> tracked_frame_states(const mjData& data) const;

    /**
     * @brief Destroy the simulation core and release MuJoCo resources.
     */
    ~MujocoSimCore();

   private:
    /**
     * @brief Load the MuJoCo model and initialize mjData.
     * @param config Static construction configuration.
     */
    void initialize_model(const Config& config);

    /**
     * @brief Build joint, actuator, and publication index mappings.
     */
    void initialize_joint_mappings();

    /**
     * @brief Resolve configured tracked frames to MuJoCo bodies or sites.
     * @param tracked_frame_names Configured frame names to resolve.
     */
    void initialize_tracked_frames(const std::vector<std::string>& tracked_frame_names);

    /**
     * @brief Validate and store configured force/torque sensor slices.
     * @param config Static construction configuration.
     */
    void initialize_ft_sensors(const Config& config);

    /**
     * @brief Initialize command buffers and control-state defaults.
     */
    void initialize_control_state();
    std::unique_ptr<mjModel, void (*)(mjModel*)> _mj_model;
    std::unique_ptr<mjData, void (*)(mjData*)> _mj_data;
    std::vector<std::string> _joint_names;
    std::unordered_map<std::string, std::size_t> _joint_state_indices_by_name;
    std::vector<std::size_t> _joint_position_indices;
    std::vector<std::size_t> _joint_velocity_indices;
    std::vector<std::size_t> _joint_effort_indices;
    std::unordered_map<std::string, std::size_t> _control_indices_by_name;
    ControlMode _control_mode;
    std::vector<ForceTorqueSensorInfo> _ft_sensor_configs;
    std::vector<TrackedFrameInfo> _tracked_frame_configs;
    std::vector<double> _commanded_effort;

    // params
    bool _enable_gravity_compensation = false;
    double _joint_damping_gain = 0.0;
    int _sim_frequency = 1000;
    bool _has_fixed_cameras = false;
    mutable std::recursive_mutex _state_mutex;
};
