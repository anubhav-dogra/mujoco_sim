#include <mujoco_core/MujocoSimCore.h>
#include <stdexcept>

MujocoSimCore::MujocoSimCore(const Config& config)
    : _mj_model(nullptr, mj_deleteModel),
      _mj_data(nullptr, mj_deleteData),
      _control_mode(config.control_mode),
      _enable_gravity_compensation(config.enable_gravity_compensation),
      _joint_damping_gain(config.joint_damping_gain),
      _sim_frequency(config.simulation_frequency) {
    initialize_model(config);
    initialize_joint_mappings();
    initialize_tracked_frames(config.tracked_frame_names);
    initialize_ft_sensors(config);
    initialize_control_state();
}

void MujocoSimCore::initialize_model(const Config& config) {
    // load all plugins before loading model!!
    // mj_loadPluginLibrary("/home/terabotics/mujoco_ws/src/mujoco/mujoco-3.2.3/bin/mujoco_plugin/libelasticity.so");
    mj_loadAllPluginLibraries(config.plugin_directory.c_str(), nullptr);

    // Load the robot model
    char error[1000] = "Could not load binary model";
    _mj_model.reset(mj_loadXML(config.xml_location.c_str(), nullptr, error, 1000));

    if (!_mj_model) {
        throw std::runtime_error(error);
    }
    _mj_model->opt.timestep = 1.0 / _sim_frequency;  // Update timestep to match simulation frequency
    _mj_data.reset(mj_makeData(_mj_model.get()));    // Initialize joint state

    // Initialize the simulation state from the first keyframe when available.
    if (_mj_model->nkey > 0) {
        mj_resetDataKeyframe(_mj_model.get(), _mj_data.get(), 0);
        mj_forward(_mj_model.get(), _mj_data.get());
    } else {
        mj_step(_mj_model.get(), _mj_data.get());
    }

    _has_fixed_cameras = _mj_model->ncam > 0;
}

void MujocoSimCore::initialize_control_state() {
    if (!_mj_model || !_mj_data) {
        throw std::runtime_error("MuJoCo model or data is not initialized.");
    }

    _commanded_effort.assign(_mj_model->nu, 0.0);
    if (_control_mode == POSITION) {
        for (int i = 0; i < _mj_model->nu; ++i) {
            if (_mj_model->actuator_trntype[i] == mjTRN_JOINT ||
                _mj_model->actuator_trntype[i] == mjTRN_JOINTINPARENT) {
                const int joint_id = _mj_model->actuator_trnid[2 * i];
                const int qpos_adr = _mj_model->jnt_qposadr[joint_id];
                _mj_data->ctrl[i] = _mj_data->qpos[qpos_adr];
            } else {
                _mj_data->ctrl[i] = 0.0;
            }
        }
    }
}

void MujocoSimCore::initialize_joint_mappings() {
    _joint_names.clear();
    _joint_position_indices.clear();
    _joint_velocity_indices.clear();
    _joint_effort_indices.clear();
    _control_indices_by_name.clear();

    for (int i = 0; i < _mj_model->njnt; i++) {
        const auto joint_name = std::string_view(_mj_model->names + _mj_model->name_jntadr[i]);
        if (_mj_model->jnt_type[i] != mjtJoint::mjJNT_HINGE && _mj_model->jnt_type[i] != mjtJoint::mjJNT_SLIDE) {
            continue;
        }
        _joint_names.emplace_back(joint_name);
        _joint_position_indices.emplace_back(_mj_model->jnt_qposadr[i]);
        _joint_velocity_indices.emplace_back(_mj_model->jnt_dofadr[i]);
    }

    for (int i = 0; i < _mj_model->nu; i++) {
        const auto actuator_name = std::string_view(_mj_model->names + _mj_model->name_actuatoradr[i]);
        _joint_effort_indices.emplace_back(i);
        _control_indices_by_name.emplace(actuator_name, i);
        if (_mj_model->actuator_trntype[i] == mjTRN_JOINT || _mj_model->actuator_trntype[i] == mjTRN_JOINTINPARENT) {
            const int joint_id = _mj_model->actuator_trnid[2 * i];
            const auto joint_name = std::string_view(_mj_model->names + _mj_model->name_jntadr[joint_id]);
            _control_indices_by_name.emplace(joint_name, i);
        }
    }
}

void MujocoSimCore::initialize_tracked_frames(const std::vector<std::string>& tracked_frame_names) {
    _tracked_frame_configs.clear();
    _tracked_frame_configs.reserve(tracked_frame_names.size());

    for (const auto& tracked_frame_name : tracked_frame_names) {
        TrackedFrameInfo tracked_frame;
        tracked_frame.name = tracked_frame_name;

        const int body_id = mj_name2id(_mj_model.get(), mjOBJ_BODY, tracked_frame_name.c_str());
        const int site_id = mj_name2id(_mj_model.get(), mjOBJ_SITE, tracked_frame_name.c_str());

        if (body_id >= 0) {
            tracked_frame.type = TrackedFrameType::BODY;
            tracked_frame.object_id = body_id;
            _tracked_frame_configs.emplace_back(std::move(tracked_frame));
        } else if (site_id >= 0) {
            tracked_frame.type = TrackedFrameType::SITE;
            tracked_frame.object_id = site_id;
            _tracked_frame_configs.emplace_back(std::move(tracked_frame));
        } else {
            throw std::runtime_error("Tracked frame '" + tracked_frame_name +
                                     "' was not found as a MuJoCo body or site.");
        }
    }
}

void MujocoSimCore::initialize_ft_sensors(const Config& config) {
    if (config.ft_force_sensor_names.size() != config.ft_torque_sensor_names.size() ||
        config.ft_force_sensor_names.size() != config.ft_sensor_frame_ids.size()) {
        throw std::runtime_error("FT sensor configuration lists must have matching lengths.");
    }

    _ft_sensor_configs.clear();
    _ft_sensor_configs.reserve(config.ft_sensor_frame_ids.size());

    for (std::size_t i = 0; i < config.ft_sensor_frame_ids.size(); ++i) {
        ForceTorqueSensorInfo ft_sensor_info;
        ft_sensor_info.frame_id = config.ft_sensor_frame_ids[i];
        ft_sensor_info.force_sensor_name = config.ft_force_sensor_names[i];
        ft_sensor_info.torque_sensor_name = config.ft_torque_sensor_names[i];
        ft_sensor_info.force_sensor_id =
            mj_name2id(_mj_model.get(), mjOBJ_SENSOR, ft_sensor_info.force_sensor_name.c_str());
        ft_sensor_info.torque_sensor_id =
            mj_name2id(_mj_model.get(), mjOBJ_SENSOR, ft_sensor_info.torque_sensor_name.c_str());

        if (ft_sensor_info.force_sensor_id < 0) {
            throw std::runtime_error("Force sensor '" + ft_sensor_info.force_sensor_name + "' was not found.");
        }
        if (ft_sensor_info.torque_sensor_id < 0) {
            throw std::runtime_error("Torque sensor '" + ft_sensor_info.torque_sensor_name + "' was not found.");
        }

        if (_mj_model->sensor_dim[ft_sensor_info.force_sensor_id] != 3) {
            throw std::runtime_error("Force sensor '" + ft_sensor_info.force_sensor_name +
                                     "' does not expose a 3-axis output.");
        }
        if (_mj_model->sensor_dim[ft_sensor_info.torque_sensor_id] != 3) {
            throw std::runtime_error("Torque sensor '" + ft_sensor_info.torque_sensor_name +
                                     "' does not expose a 3-axis output.");
        }

        ft_sensor_info.force_sensor_adr = _mj_model->sensor_adr[ft_sensor_info.force_sensor_id];
        ft_sensor_info.torque_sensor_adr = _mj_model->sensor_adr[ft_sensor_info.torque_sensor_id];
        _ft_sensor_configs.emplace_back(std::move(ft_sensor_info));
    }
}

mjModel* MujocoSimCore::model() const { return _mj_model.get(); }

mjData* MujocoSimCore::data() const { return _mj_data.get(); }

std::recursive_mutex& MujocoSimCore::state_mutex() const { return _state_mutex; }

void MujocoSimCore::step() {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data) {
        throw std::runtime_error("MuJoCo model or data is not initialized.");
    }

    if (_control_mode == TORQUE) {
        for (int i = 0; i < _mj_model->nu; ++i) {
            double control_effort = _commanded_effort[static_cast<std::size_t>(i)];
            if (_enable_gravity_compensation && i < _mj_model->nv) {
                control_effort += _mj_data->qfrc_bias[i];
            }
            if (_joint_damping_gain != 0.0 && i < _mj_model->nv) {
                control_effort -= _joint_damping_gain * _mj_data->qvel[i];
            }
            _mj_data->ctrl[i] = control_effort;
        }
    }

    mj_step(_mj_model.get(), _mj_data.get());
}

void MujocoSimCore::reset() {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data) {
        throw std::runtime_error("MuJoCo model or data is not initialized.");
    }

    if (_mj_model->nkey > 0) {
        mj_resetDataKeyframe(_mj_model.get(), _mj_data.get(), 0);
        mj_forward(_mj_model.get(), _mj_data.get());
    } else {
        mj_resetData(_mj_model.get(), _mj_data.get());
    }

    std::fill(_commanded_effort.begin(), _commanded_effort.end(), 0.0);
    initialize_control_state();
}

bool MujocoSimCore::has_fixed_cameras() const { return _has_fixed_cameras; }

void MujocoSimCore::set_position_command(std::size_t actuator_id, double value) {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data || actuator_id >= static_cast<std::size_t>(_mj_model->nu)) {
        throw std::out_of_range("Invalid actuator index for position command.");
    }
    _mj_data->ctrl[actuator_id] = value;
}

void MujocoSimCore::set_effort_command(std::size_t actuator_id, double value) {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data || actuator_id >= _commanded_effort.size()) {
        throw std::out_of_range("Invalid actuator index for effort command.");
    }
    _commanded_effort[actuator_id] = value;
}

const std::vector<std::string>& MujocoSimCore::joint_names() const { return _joint_names; }

const std::vector<std::size_t>& MujocoSimCore::joint_position_indices() const { return _joint_position_indices; }

const std::vector<std::size_t>& MujocoSimCore::joint_velocity_indices() const { return _joint_velocity_indices; }

const std::vector<std::size_t>& MujocoSimCore::joint_effort_indices() const { return _joint_effort_indices; }

const std::unordered_map<std::string, std::size_t>& MujocoSimCore::control_indices_by_name() const {
    return _control_indices_by_name;
}

const std::vector<MujocoSimCore::ForceTorqueSensorInfo>& MujocoSimCore::ft_sensor_configs() const {
    return _ft_sensor_configs;
}

const std::vector<MujocoSimCore::TrackedFrameInfo>& MujocoSimCore::tracked_frame_configs() const {
    return _tracked_frame_configs;
}

std::vector<MujocoSimCore::ForceTorqueSensorWrench> MujocoSimCore::force_torque_states() const {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data) {
        throw std::runtime_error("MuJoCo model or data is not initialized.");
    }

    return force_torque_states(*_mj_data);
}

std::vector<MujocoSimCore::ForceTorqueSensorWrench> MujocoSimCore::force_torque_states(const mjData& data) const {
    if (!_mj_model) {
        throw std::runtime_error("MuJoCo model is not initialized.");
    }

    std::vector<ForceTorqueSensorWrench> force_torque_states;
    force_torque_states.reserve(_ft_sensor_configs.size());

    for (const auto& ft_sensor_config : _ft_sensor_configs) {
        ForceTorqueSensorWrench force_torque_state;
        force_torque_state.frame_id = ft_sensor_config.frame_id;

        if (ft_sensor_config.force_sensor_id >= 0 && ft_sensor_config.torque_sensor_id >= 0 &&
            ft_sensor_config.force_sensor_adr >= 0 && ft_sensor_config.torque_sensor_adr >= 0) {
            force_torque_state.available = true;
            force_torque_state.force = {data.sensordata[ft_sensor_config.force_sensor_adr],
                                        data.sensordata[ft_sensor_config.force_sensor_adr + 1],
                                        data.sensordata[ft_sensor_config.force_sensor_adr + 2]};
            force_torque_state.torque = {data.sensordata[ft_sensor_config.torque_sensor_adr],
                                         data.sensordata[ft_sensor_config.torque_sensor_adr + 1],
                                         data.sensordata[ft_sensor_config.torque_sensor_adr + 2]};
        }

        force_torque_states.emplace_back(std::move(force_torque_state));
    }

    return force_torque_states;
}

std::vector<MujocoSimCore::TrackedFrameState> MujocoSimCore::tracked_frame_states() const {
    std::lock_guard<std::recursive_mutex> lock(_state_mutex);
    if (!_mj_model || !_mj_data) {
        throw std::runtime_error("MuJoCo model or data is not initialized.");
    }

    return tracked_frame_states(*_mj_data);
}

std::vector<MujocoSimCore::TrackedFrameState> MujocoSimCore::tracked_frame_states(const mjData& data) const {
    if (!_mj_model) {
        throw std::runtime_error("MuJoCo model is not initialized.");
    }

    std::vector<TrackedFrameState> tracked_frame_states;
    tracked_frame_states.reserve(_tracked_frame_configs.size());

    for (const auto& tracked_frame_config : _tracked_frame_configs) {
        TrackedFrameState tracked_frame_state;
        tracked_frame_state.name = tracked_frame_config.name;

        if (tracked_frame_config.type == TrackedFrameType::BODY) {
            const mjtNum* body_position = data.xpos + 3 * tracked_frame_config.object_id;
            const mjtNum* body_orientation = data.xquat + 4 * tracked_frame_config.object_id;
            tracked_frame_state.position = {body_position[0], body_position[1], body_position[2]};
            tracked_frame_state.orientation = {body_orientation[0], body_orientation[1], body_orientation[2],
                                               body_orientation[3]};
        } else {
            const mjtNum* site_position = data.site_xpos + 3 * tracked_frame_config.object_id;
            const mjtNum* site_rotation = data.site_xmat + 9 * tracked_frame_config.object_id;
            mjtNum site_quaternion[4];
            mju_mat2Quat(site_quaternion, site_rotation);
            tracked_frame_state.position = {site_position[0], site_position[1], site_position[2]};
            tracked_frame_state.orientation = {site_quaternion[0], site_quaternion[1], site_quaternion[2],
                                               site_quaternion[3]};
        }

        tracked_frame_states.emplace_back(std::move(tracked_frame_state));
    }

    return tracked_frame_states;
}

MujocoSimCore::~MujocoSimCore() = default;
