#include <gtest/gtest.h>
#include <filesystem>
#include <mujoco_core/MujocoSimCore.h>
#include <string>
#include <vector>
#include <cmath>

namespace {
std::string repo_root() { return MUJOCO_ROS2_INTERFACE_SOURCE_DIR; }
std::string model_path() { return repo_root() + "/../mujoco_models/models/planar_2r.xml"; }
std::string plugin_dir() { return repo_root() + "/.pixi/envs/default/bin/mujoco_plugin"; }

MujocoSimCore::Config make_config() {
    MujocoSimCore::Config config;
    config.control_mode = POSITION;
    config.enable_gravity_compensation = false;
    config.visualization_enabled = false;
    config.xml_location = model_path();
    config.plugin_directory = plugin_dir();
    config.tracked_frame_names = {"end_effector_site"};
    return config;
}

MujocoSimCore::Config make_torque_config() {
    MujocoSimCore::Config config = make_config();
    config.control_mode = TORQUE;
    config.enable_gravity_compensation = false;
    config.joint_damping_gain = 0.0;
    return config;
}

MujocoSimCore::Config make_ft_config() {
    MujocoSimCore::Config config = make_config();
    config.ft_force_sensor_names = {"end_effector_force_sensor"};
    config.ft_torque_sensor_names = {"end_effector_torque_sensor"};
    config.ft_sensor_frame_ids = {"end_effector_site"};
    return config;
}

MujocoSimCore::Config make_invalid_ft_config() {
    MujocoSimCore::Config config = make_config();
    config.ft_force_sensor_names = {"missing_force_sensor"};
    config.ft_torque_sensor_names = {"end_effector_torque_sensor"};
    config.ft_sensor_frame_ids = {"end_effector_site"};
    return config;
}

MujocoSimCore::Config make_invalid_tracked_frame_config() {
    MujocoSimCore::Config config = make_config();
    config.tracked_frame_names = {"missing_frame"};
    return config;
}

TEST(MujocoSimCoreTest, ConstructsAndLoadsModel) {
    MujocoSimCore sim_core(make_config());
    ASSERT_NE(sim_core.model(), nullptr);
    ASSERT_NE(sim_core.data(), nullptr);
    EXPECT_EQ(sim_core.joint_names().size(), 2u);
}

TEST(MujocoSimCoreTest, BuildsJointControlMappings) {
    MujocoSimCore sim_core(make_config());
    const auto& control_indices = sim_core.control_indices_by_name();
    EXPECT_TRUE(control_indices.count("joint1"));
    EXPECT_TRUE(control_indices.count("joint2"));
    EXPECT_TRUE(control_indices.count("joint1_actuator"));
    EXPECT_TRUE(control_indices.count("joint2_actuator"));
}

TEST(MujocoSimCoreTest, ResolvesTrackedFrameNames) {
    MujocoSimCore sim_core(make_config());
    const auto& tracked_frames = sim_core.tracked_frame_configs();
    ASSERT_EQ(tracked_frames.size(), 1u);
    EXPECT_EQ(tracked_frames[0].name, "end_effector_site");

    const auto& tracked_states = sim_core.tracked_frame_states();
    ASSERT_EQ(tracked_states.size(), 1u);
}

TEST(MujocoSimCoreTest, PositionCommandWritesControlTargetImmediately) {
    MujocoSimCore sim_core(make_config());

    const auto& control_indices = sim_core.control_indices_by_name();
    ASSERT_TRUE(control_indices.count("joint1_actuator"));
    ASSERT_TRUE(control_indices.count("joint2_actuator"));

    auto* data = sim_core.data();
    ASSERT_NE(data, nullptr);

    sim_core.set_position_command(control_indices.at("joint1_actuator"), 0.25);
    sim_core.set_position_command(control_indices.at("joint2_actuator"), -0.4);

    EXPECT_DOUBLE_EQ(data->ctrl[control_indices.at("joint1_actuator")], 0.25);
    EXPECT_DOUBLE_EQ(data->ctrl[control_indices.at("joint2_actuator")], -0.4);
}

TEST(MujocoSimCoreTest, EffortCommandAppliesOnStepInTorqueMode) {
    MujocoSimCore sim_core(make_torque_config());

    const auto& control_indices = sim_core.control_indices_by_name();
    ASSERT_TRUE(control_indices.count("joint1_actuator"));
    ASSERT_TRUE(control_indices.count("joint2_actuator"));

    const auto joint1_index = control_indices.at("joint1_actuator");
    const auto joint2_index = control_indices.at("joint2_actuator");

    auto* data = sim_core.data();
    ASSERT_NE(data, nullptr);

    sim_core.set_effort_command(joint1_index, 0.6);
    sim_core.set_effort_command(joint2_index, -0.2);
    sim_core.step();

    EXPECT_NEAR(data->ctrl[joint1_index], 0.6, 1e-12);
    EXPECT_NEAR(data->ctrl[joint2_index], -0.2, 1e-12);
}

TEST(MujocoSimCoreTest, ResolvesForceTorqueSensorConfiguration) {
    MujocoSimCore sim_core(make_ft_config());

    const auto& ft_configs = sim_core.ft_sensor_configs();
    ASSERT_EQ(ft_configs.size(), 1u);
    EXPECT_EQ(ft_configs[0].frame_id, "end_effector_site");
    EXPECT_EQ(ft_configs[0].force_sensor_name, "end_effector_force_sensor");
    EXPECT_EQ(ft_configs[0].torque_sensor_name, "end_effector_torque_sensor");
    EXPECT_GE(ft_configs[0].force_sensor_id, 0);
    EXPECT_GE(ft_configs[0].torque_sensor_id, 0);
    EXPECT_GE(ft_configs[0].force_sensor_adr, 0);
    EXPECT_GE(ft_configs[0].torque_sensor_adr, 0);

    const auto ft_states = sim_core.force_torque_states();
    ASSERT_EQ(ft_states.size(), 1u);
    EXPECT_EQ(ft_states[0].frame_id, "end_effector_site");
    EXPECT_TRUE(ft_states[0].available);
}

TEST(MujocoSimCoreTest, InvalidForceSensorNameThrows) {
    EXPECT_THROW({ MujocoSimCore sim_core(make_invalid_ft_config()); }, std::runtime_error);
}

TEST(MujocoSimCoreTest, InvalidTrackedFrameNameThrows) {
    EXPECT_THROW({ MujocoSimCore sim_core(make_invalid_tracked_frame_config()); }, std::runtime_error);
}

TEST(MujocoSimCoreTest, ResetRestoresInitialState) {
    MujocoSimCore sim_core(make_config());

    const auto* model = sim_core.model();
    auto* data = sim_core.data();

    ASSERT_NE(model, nullptr);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(sim_core.joint_names().size(), 2u);

    const std::vector<double> initial_position(data->qpos, data->qpos + model->nq);
    const std::vector<double> initial_ctrl(data->ctrl, data->ctrl + model->nu);

    const auto& control_indices = sim_core.control_indices_by_name();
    ASSERT_TRUE(control_indices.count("joint1_actuator"));
    ASSERT_TRUE(control_indices.count("joint2_actuator"));

    sim_core.set_position_command(control_indices.at("joint1_actuator"), 0.5);
    sim_core.set_position_command(control_indices.at("joint2_actuator"), -0.5);

    for (int i = 0; i < 10000; ++i) {
        sim_core.step();
    }

    bool state_changed = false;
    for (int i = 0; i < model->nq; ++i) {
        if (std::abs(data->qpos[i] - initial_position[i]) > 1e-6) {
            state_changed = true;
            break;
        }
    }
    EXPECT_TRUE(state_changed);

    sim_core.reset();
    for (int i = 0; i < model->nq; ++i) {
        EXPECT_NEAR(data->qpos[i], initial_position[i], 1e-9);
    }

    for (int i = 0; i < model->nu; ++i) {
        EXPECT_NEAR(data->ctrl[i], initial_ctrl[i], 1e-9);
    }
}

}  // namespace
