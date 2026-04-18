#pragma once

#include <cstddef>
#include <atomic>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <thread>
#include <vector>

// for definiton, lighter then calling the full header here
class MujocoSimCore;

namespace mujoco_ros2_driver {

// ros2 control manager specifics
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class MujocoRos2Driver : public hardware_interface::SystemInterface {
   public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    ~MujocoRos2Driver() override;

   private:
    void start_sim_thread();
    void stop_sim_thread();
    void run_sim_loop();
    void start_viewer_thread();
    void stop_viewer_thread();
    void run_viewer_loop();

    struct JointBinding {
        std::string ros_joint_name;
        std::string mujoco_state_joint_name;
        std::string mujoco_control_name;
        std::size_t qpos_index = 0;
        std::size_t qvel_index = 0;
        std::size_t control_index = 0;
    };

    hardware_interface::HardwareInfo info_;
    std::vector<std::vector<double>> hw_joint_commands_;
    std::vector<std::vector<double>> hw_joint_states_;
    std::vector<std::vector<double>> hw_sensor_states_;

    // mujoco_core
    std::unique_ptr<MujocoSimCore> sim_core_;
    std::string xml_location{};
    std::string control_mode{};
    int control_frequency = 1000;
    bool visualization_enabled = false;

    // sharing variables
    std::vector<JointBinding> joint_bindings_;
    std::vector<std::size_t> ros_to_ft_sensor_index_;
    std::atomic<bool> sim_running_{false};
    std::thread sim_thread_;
    std::chrono::nanoseconds sim_step_period_{std::chrono::milliseconds(1)};
    std::atomic<bool> viewer_running_{false};
    std::thread viewer_thread_;
    std::atomic<bool> viewer_paused_{false};
    std::atomic<bool> viewer_reset_requested_{false};
};
}  // namespace mujoco_ros2_driver
