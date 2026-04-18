#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cctype>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iterator>
#include <limits>
#include <memory>
#include <mujoco_ros2_driver/mujoco_ros2_driver.hpp>
#include <mutex>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>
#include <mujoco_core/MujocoSimCore.h>
#include <mujoco_viewer/MujocoViewer.h>
#include <mujoco_viewer/ViewerControls.h>
#include <rclcpp/rclcpp.hpp>

namespace {

bool parse_bool_param(const std::string& value) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on";
}

std::string get_joint_param_or_default(const hardware_interface::ComponentInfo& joint, const char* param_name,
                                       const std::string& default_value) {
    const auto it = joint.parameters.find(param_name);
    return it != joint.parameters.end() ? it->second : default_value;
}

const char* control_mode_to_string(const ControlMode mode) {
    switch (mode) {
        case POSITION:
            return "position";
        case VELOCITY:
            return "velocity";
        case TORQUE:
            return "torque";
        default:
            return "unknown";
    }
}

}  // namespace

namespace mujoco_ros2_driver {
CallbackReturn MujocoRos2Driver::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    info_ = info;
    hw_joint_states_.resize(info_.joints.size());
    hw_joint_commands_.resize(info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        hw_joint_states_[i].resize(info_.joints[i].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
        hw_joint_commands_[i].resize(info_.joints[i].command_interfaces.size(),
                                     std::numeric_limits<double>::quiet_NaN());
    }
    hw_sensor_states_.resize(info_.sensors.size());
    for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
        hw_sensor_states_[i].resize(info_.sensors[i].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
    }

    joint_bindings_.resize(info_.joints.size());
    ros_to_ft_sensor_index_.resize(info_.sensors.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoRos2Driver::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        for (std::size_t j = 0; j < info_.joints[i].state_interfaces.size(); ++j) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[j].name, &hw_joint_states_[i][j]));
        }
    }
    for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
        for (std::size_t j = 0; j < info_.sensors[i].state_interfaces.size(); ++j) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_[i][j]));
        }
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoRos2Driver::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        for (std::size_t j = 0; j < info_.joints[i].command_interfaces.size(); ++j) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, info_.joints[i].command_interfaces[j].name, &hw_joint_commands_[i][j]));
        }
    }
    return command_interfaces;
}

CallbackReturn MujocoRos2Driver::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto logger = rclcpp::get_logger("MujocoRos2Driver");
    stop_viewer_thread();
    stop_sim_thread();
    sim_core_.reset();

    if (info_.hardware_parameters.find("xml_location") == info_.hardware_parameters.end()) {
        RCLCPP_ERROR(logger, "mujoco xml model location is not specified");
        return CallbackReturn::ERROR;
    } else {
        xml_location = info_.hardware_parameters["xml_location"];
    }
    if (info_.hardware_parameters.find("simulation_frequency") == info_.hardware_parameters.end() &&
        info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
        control_frequency = 100;
        RCLCPP_INFO(logger, "simulation_frequency is not specified: setting to default [100]");
    } else if (info_.hardware_parameters.find("simulation_frequency") != info_.hardware_parameters.end()) {
        control_frequency = std::stod(info_.hardware_parameters["simulation_frequency"]);
    } else {
        control_frequency = std::stod(info_.hardware_parameters["control_frequency"]);
    }
    if (info_.hardware_parameters.find("control_mode") == info_.hardware_parameters.end()) {
        control_mode = "position";
        RCLCPP_INFO(logger, "control_mode is not specified: defaulting to 'position'");
    } else {
        control_mode = info_.hardware_parameters["control_mode"];
    }
    if (info_.hardware_parameters.find("visualization_enabled") == info_.hardware_parameters.end()) {
        visualization_enabled = false;
    } else {
        visualization_enabled = parse_bool_param(info_.hardware_parameters["visualization_enabled"]);
    }

    // build the config for mujoco
    MujocoSimCore::Config config;
    config.xml_location = xml_location;
    config.simulation_frequency = control_frequency;
    config.visualization_enabled = visualization_enabled;
    for (const auto& sensor : info_.sensors) {
        const auto force_it = sensor.parameters.find("force_sensor_name");
        const auto torque_it = sensor.parameters.find("torque_sensor_name");
        if (force_it == sensor.parameters.end() || torque_it == sensor.parameters.end()) {
            RCLCPP_ERROR(logger, "FT sensor '%s' must define 'force_sensor_name' and 'torque_sensor_name' parameters",
                         sensor.name.c_str());
            return CallbackReturn::ERROR;
        }

        const auto frame_it = sensor.parameters.find("frame_id");
        config.ft_sensor_frame_ids.push_back(frame_it != sensor.parameters.end() ? frame_it->second : sensor.name);
        config.ft_force_sensor_names.push_back(force_it->second);
        config.ft_torque_sensor_names.push_back(torque_it->second);
    }
    if (control_mode == "position") {
        config.control_mode = POSITION;
    } else if (control_mode == "torque") {
        config.control_mode = TORQUE;
    } else if (control_mode == "velocity") {
        config.control_mode = VELOCITY;
    } else {
        RCLCPP_ERROR(logger, "Unsupported control_mode: %s", control_mode.c_str());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger, "Configuring MuJoCo driver: xml='%s', mode='%s', sim_frequency=%d Hz, visualization=%s",
                config.xml_location.c_str(), control_mode_to_string(config.control_mode), config.simulation_frequency,
                config.visualization_enabled ? "true" : "false");

    sim_core_ = std::make_unique<MujocoSimCore>(config);
    const int sim_frequency = std::max(1, config.simulation_frequency);
    sim_step_period_ = std::chrono::nanoseconds(static_cast<long long>(1e9 / sim_frequency));

    // Resolve separate MuJoCo state-joint and control-target bindings for each ROS joint.
    const auto& control_indices = sim_core_->control_indices_by_name();
    const auto& joint_state_indices = sim_core_->joint_state_indices_by_name();
    const auto& joint_qpos_indices = sim_core_->joint_position_indices();
    const auto& joint_qvel_indices = sim_core_->joint_velocity_indices();
    const auto& ft_sensor_configs = sim_core_->ft_sensor_configs();
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        auto& binding = joint_bindings_[i];
        binding.ros_joint_name = info_.joints[i].name;
        binding.mujoco_state_joint_name =
            get_joint_param_or_default(info_.joints[i], "state_joint_name", info_.joints[i].name);
        binding.mujoco_control_name = get_joint_param_or_default(info_.joints[i], "control_name", info_.joints[i].name);

        const auto control_it = control_indices.find(binding.mujoco_control_name);
        if (control_it == control_indices.end()) {
            RCLCPP_ERROR(logger, "Joint '%s' not found in MuJoCo control map using control target '%s'",
                         binding.ros_joint_name.c_str(), binding.mujoco_control_name.c_str());
            return CallbackReturn::ERROR;
        }
        binding.control_index = control_it->second;

        const auto joint_state_it = joint_state_indices.find(binding.mujoco_state_joint_name);
        if (joint_state_it == joint_state_indices.end()) {
            RCLCPP_ERROR(logger, "Joint '%s' not found in MuJoCo joint state map using state joint '%s'",
                         binding.ros_joint_name.c_str(), binding.mujoco_state_joint_name.c_str());
            return CallbackReturn::ERROR;
        }
        const std::size_t joint_state_index = joint_state_it->second;
        binding.qpos_index = joint_qpos_indices[joint_state_index];
        binding.qvel_index = joint_qvel_indices[joint_state_index];

        RCLCPP_INFO(logger, "Mapped joint '%s' -> state_joint='%s' [qpos=%zu qvel=%zu], control_target='%s' [ctrl=%zu]",
                    binding.ros_joint_name.c_str(), binding.mujoco_state_joint_name.c_str(), binding.qpos_index,
                    binding.qvel_index, binding.mujoco_control_name.c_str(), binding.control_index);
    }
    for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
        const auto frame_it = info_.sensors[i].parameters.find("frame_id");
        const std::string frame_id =
            frame_it != info_.sensors[i].parameters.end() ? frame_it->second : info_.sensors[i].name;

        const auto ft_it = std::find_if(ft_sensor_configs.begin(), ft_sensor_configs.end(),
                                        [&frame_id](const auto& cfg) { return cfg.frame_id == frame_id; });
        if (ft_it == ft_sensor_configs.end()) {
            RCLCPP_ERROR(logger, "FT sensor '%s' with frame_id '%s' not found in MuJoCo FT configuration",
                         info_.sensors[i].name.c_str(), frame_id.c_str());
            return CallbackReturn::ERROR;
        }
        ros_to_ft_sensor_index_[i] = static_cast<std::size_t>(std::distance(ft_sensor_configs.begin(), ft_it));
        RCLCPP_INFO(logger, "Mapped FT sensor '%s' -> frame_id='%s', ft_index=%zu", info_.sensors[i].name.c_str(),
                    frame_id.c_str(), ros_to_ft_sensor_index_[i]);
    }

    RCLCPP_INFO(logger, "MuJoCo driver configured successfully with %zu joints and %zu FT sensors", info_.joints.size(),
                info_.sensors.size());

    return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoRos2Driver::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto logger = rclcpp::get_logger("MujocoRos2Driver");
    if (!sim_core_) {
        RCLCPP_ERROR(logger, "Cannot activate driver without an initialized MuJoCo core");
        return CallbackReturn::ERROR;
    }

    {
        std::lock_guard<std::recursive_mutex> lock(sim_core_->state_mutex());
        sim_core_->reset();
    }
    viewer_paused_.store(false, std::memory_order_release);
    viewer_reset_requested_.store(false, std::memory_order_release);
    start_sim_thread();
    if (visualization_enabled) {
        start_viewer_thread();
    }
    RCLCPP_INFO(logger, "Activated MuJoCo driver: sim thread running, visualization=%s",
                visualization_enabled ? "true" : "false");
    return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoRos2Driver::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    stop_viewer_thread();
    stop_sim_thread();
    RCLCPP_INFO(rclcpp::get_logger("MujocoRos2Driver"), "Deactivated MuJoCo driver");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoRos2Driver::read(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
    if (!sim_core_) {
        return hardware_interface::return_type::ERROR;
    }

    std::lock_guard<std::recursive_mutex> lock(sim_core_->state_mutex());
    const auto* data = sim_core_->data();
    const auto ft_states = sim_core_->force_torque_states(*data);

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& binding = joint_bindings_[i];

        for (std::size_t j = 0; j < info_.joints[i].state_interfaces.size(); ++j) {
            const auto interface_name = info_.joints[i].state_interfaces[j].name;
            if (interface_name == hardware_interface::HW_IF_POSITION) {
                hw_joint_states_[i][j] = data->qpos[binding.qpos_index];
            } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
                hw_joint_states_[i][j] = data->qvel[binding.qvel_index];
            } else if (interface_name == hardware_interface::HW_IF_EFFORT) {
                hw_joint_states_[i][j] = data->actuator_force[binding.control_index];
            }
        }
    }
    for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
        const auto& ft_state = ft_states[ros_to_ft_sensor_index_[i]];
        for (std::size_t j = 0; j < info_.sensors[i].state_interfaces.size(); ++j) {
            const auto& interface_name = info_.sensors[i].state_interfaces[j].name;
            if (interface_name == "force.x") {
                hw_sensor_states_[i][j] = ft_state.force[0];
            } else if (interface_name == "force.y") {
                hw_sensor_states_[i][j] = ft_state.force[1];
            } else if (interface_name == "force.z") {
                hw_sensor_states_[i][j] = ft_state.force[2];
            } else if (interface_name == "torque.x") {
                hw_sensor_states_[i][j] = ft_state.torque[0];
            } else if (interface_name == "torque.y") {
                hw_sensor_states_[i][j] = ft_state.torque[1];
            } else if (interface_name == "torque.z") {
                hw_sensor_states_[i][j] = ft_state.torque[2];
            }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoRos2Driver::write(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
    if (!sim_core_) {
        return hardware_interface::return_type::ERROR;
    }
    std::lock_guard<std::recursive_mutex> lock(sim_core_->state_mutex());

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& binding = joint_bindings_[i];

        for (std::size_t j = 0; j < info_.joints[i].command_interfaces.size(); ++j) {
            const auto& interface_name = info_.joints[i].command_interfaces[j].name;
            const double command_value = hw_joint_commands_[i][j];

            if (!std::isfinite(command_value)) {
                continue;
            }

            if (interface_name == hardware_interface::HW_IF_POSITION) {
                sim_core_->set_position_command(binding.control_index, command_value);
            }
            if (interface_name == hardware_interface::HW_IF_EFFORT) {
                sim_core_->set_effort_command(binding.control_index, command_value);
            }
        }
    }
    return hardware_interface::return_type::OK;
}

void MujocoRos2Driver::start_sim_thread() {
    stop_sim_thread();
    sim_running_.store(true, std::memory_order_release);
    sim_thread_ = std::thread(&MujocoRos2Driver::run_sim_loop, this);
}

void MujocoRos2Driver::stop_sim_thread() {
    sim_running_.store(false, std::memory_order_release);
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
}

void MujocoRos2Driver::run_sim_loop() {
    while (sim_running_.load(std::memory_order_acquire)) {
        auto step_start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::recursive_mutex> lock(sim_core_->state_mutex());
            if (viewer_reset_requested_.exchange(false, std::memory_order_acq_rel)) {
                sim_core_->reset();
            }
            if (!viewer_paused_.load(std::memory_order_acquire)) {
                sim_core_->step();
            }
        }
        std::this_thread::sleep_until(step_start + sim_step_period_);
    }
}

void MujocoRos2Driver::start_viewer_thread() {
    stop_viewer_thread();
    viewer_running_.store(true, std::memory_order_release);
    viewer_thread_ = std::thread(&MujocoRos2Driver::run_viewer_loop, this);
}

void MujocoRos2Driver::stop_viewer_thread() {
    viewer_running_.store(false, std::memory_order_release);
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void MujocoRos2Driver::run_viewer_loop() {
    if (!sim_core_) {
        return;
    }

    MujocoViewer viewer;
    viewer.initialize(*sim_core_->model(), false, &viewer_paused_, &viewer_reset_requested_);
    viewer.set_camera_properties({0.0, 0.0, 0.5}, 2.5, 135.0, -30.0, false);

    std::unique_ptr<mjData, void (*)(mjData*)> render_data(mj_makeData(sim_core_->model()), mj_deleteData);
    if (!render_data) {
        throw std::runtime_error("Failed to allocate render mjData snapshot for driver viewer.");
    }

    while (viewer_running_.load(std::memory_order_acquire) && !viewer.should_close()) {
        ViewerStatus viewer_status;
        viewer_status.control_mode = control_mode.c_str();
        viewer_status.paused = viewer_paused_.load(std::memory_order_acquire);
        viewer_status.collision_visible = viewer.collision_visible();
        viewer_status.contacts_visible = viewer.contacts_visible();
        viewer_status.control_panel_available = viewer.control_panel_available();
        viewer_status.control_panel_visible = viewer.control_panel_visible();
        viewer_status.jog_enabled = viewer.jog_enabled();
        viewer_status.control_slider_count = static_cast<int>(viewer.control_slider_count());
        viewer_status.simulation_frequency = control_frequency;
        viewer_status.contact_force_scale = viewer.contact_force_scale();
        viewer_status.fixed_cameras_available = sim_core_->has_fixed_cameras();

        {
            std::lock_guard<std::recursive_mutex> lock(sim_core_->state_mutex());
            mj_copyData(render_data.get(), sim_core_->model(), sim_core_->data());
        }

        viewer_status.contact_count = render_data->ncon;
        viewer_status.ft_available = false;
        for (const auto& ft_state : sim_core_->force_torque_states(*render_data)) {
            viewer_status.ft_available = viewer_status.ft_available || ft_state.available;
            viewer_status.force_torque_sensors.push_back({ft_state.frame_id, ft_state.force, ft_state.torque});
        }
        for (const auto& tracked_frame : sim_core_->tracked_frame_states(*render_data)) {
            viewer_status.tracked_frames.push_back({tracked_frame.name, tracked_frame.position});
        }

        viewer.set_status(std::move(viewer_status));
        viewer.update_scene(*render_data);
        viewer.present();
    }

    viewer_running_.store(false, std::memory_order_release);
}

MujocoRos2Driver::~MujocoRos2Driver() {
    stop_viewer_thread();
    stop_sim_thread();
}

}  // namespace mujoco_ros2_driver

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_driver::MujocoRos2Driver, hardware_interface::SystemInterface)
