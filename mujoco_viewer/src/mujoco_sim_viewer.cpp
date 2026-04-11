#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <mujoco_core/MujocoSimCore.h>
#include <mujoco_viewer/MujocoViewer.h>
#include <yaml-cpp/yaml.h>

namespace {

void print_usage() {
    std::cout << "Usage: mujoco_sim_viewer [--config <config.yaml>] [options]\n"
              << "Options:\n"
              << "  --config <config.yaml>                 Load standalone viewer configuration from YAML\n"
              << "  --xml <model.xml>                      Override model XML path\n"
              << "  --plugin-dir <mujoco_plugin_dir>       Override MuJoCo plugin directory\n"
              << "  --mode <position|torque|velocity>      Control mode for startup display/behavior\n"
              << "  --sim-frequency <hz>                   Simulation frequency in Hz\n"
              << "  --joint-damping-gain <value>           Joint damping gain used in torque mode\n"
              << "  --visualization-enabled <true|false>   Enable or disable the MuJoCo viewer\n"
              << "  --enable-gravity-compensation          Enable qfrc_bias compensation in torque mode\n"
              << "  --tracked-frame <name>                 Add a tracked body/site name, may be repeated\n"
              << "  --ft <frame_id>:<force_sensor>:<torque_sensor>\n"
              << "                                        Add one FT output, may be repeated\n"
              << "  --help, -h                             Show this help message\n";
}

const char* control_mode_label(ControlMode mode) {
    switch (mode) {
        case POSITION:
            return "POSITION";
        case TORQUE:
            return "TORQUE";
        case VELOCITY:
            return "VELOCITY";
        default:
            return "UNKNOWN";
    }
}

ControlMode parse_control_mode(const std::string& value) {
    if (value == "position") {
        return POSITION;
    }
    if (value == "torque") {
        return TORQUE;
    }
    if (value == "velocity") {
        return VELOCITY;
    }
    throw std::runtime_error("Unsupported control mode '" + value + "'. Expected one of: position, torque, velocity.");
}

int parse_positive_int(const std::string& value, std::string_view flag_name) {
    try {
        const int parsed_value = std::stoi(value);
        if (parsed_value <= 0) {
            throw std::runtime_error(std::string(flag_name) + " must be a positive integer.");
        }
        return parsed_value;
    } catch (const std::invalid_argument&) {
        throw std::runtime_error(std::string(flag_name) + " expects an integer, got '" + value + "'.");
    } catch (const std::out_of_range&) {
        throw std::runtime_error(std::string(flag_name) + " is out of range: '" + value + "'.");
    }
}

double parse_double(const std::string& value, std::string_view flag_name) {
    try {
        return std::stod(value);
    } catch (const std::invalid_argument&) {
        throw std::runtime_error(std::string(flag_name) + " expects a floating-point value, got '" + value + "'.");
    } catch (const std::out_of_range&) {
        throw std::runtime_error(std::string(flag_name) + " is out of range: '" + value + "'.");
    }
}

bool parse_bool(const std::string& value, std::string_view flag_name) {
    if (value == "true" || value == "1") {
        return true;
    }
    if (value == "false" || value == "0") {
        return false;
    }
    throw std::runtime_error(std::string(flag_name) + " expects true/false or 1/0, got '" + value + "'.");
}

void parse_force_torque_spec(const std::string& value, MujocoSimCore::Config& config) {
    const std::size_t first_separator = value.find(':');
    const std::size_t second_separator =
        value.find(':', first_separator == std::string::npos ? std::string::npos : first_separator + 1);

    if (first_separator == std::string::npos || second_separator == std::string::npos ||
        second_separator + 1 >= value.size()) {
        throw std::runtime_error("Invalid --ft value '" + value +
                                 "'. Expected <frame_id>:<force_sensor>:<torque_sensor>.");
    }

    const std::string frame_id = value.substr(0, first_separator);
    const std::string force_sensor_name = value.substr(first_separator + 1, second_separator - first_separator - 1);
    const std::string torque_sensor_name = value.substr(second_separator + 1);

    if (frame_id.empty() || force_sensor_name.empty() || torque_sensor_name.empty()) {
        throw std::runtime_error("Invalid --ft value '" + value +
                                 "'. Frame id and both sensor names must be non-empty.");
    }

    config.ft_sensor_frame_ids.push_back(frame_id);
    config.ft_force_sensor_names.push_back(force_sensor_name);
    config.ft_torque_sensor_names.push_back(torque_sensor_name);
}

void load_config_file(const std::string& config_path, MujocoSimCore::Config& config) {
    namespace fs = std::filesystem;

    if (!fs::exists(config_path)) {
        throw std::runtime_error("Standalone config file does not exist: " + config_path);
    }

    const YAML::Node root = YAML::LoadFile(config_path);
    const fs::path config_directory = fs::absolute(fs::path(config_path)).parent_path();

    if (root["xml_location"]) {
        fs::path xml_location = root["xml_location"].as<std::string>();
        if (xml_location.is_relative()) {
            xml_location = config_directory / xml_location;
        }
        config.xml_location = xml_location.lexically_normal().string();
    }
    if (root["plugin_directory"]) {
        fs::path plugin_directory = root["plugin_directory"].as<std::string>();
        if (plugin_directory.is_relative()) {
            plugin_directory = config_directory / plugin_directory;
        }
        config.plugin_directory = plugin_directory.lexically_normal().string();
    }
    if (root["control_mode"]) {
        config.control_mode = parse_control_mode(root["control_mode"].as<std::string>());
    }
    if (root["simulation_frequency"]) {
        config.simulation_frequency = root["simulation_frequency"].as<int>();
    }
    if (root["joint_damping_gain"]) {
        config.joint_damping_gain = root["joint_damping_gain"].as<double>();
    }
    if (root["enable_gravity_compensation"]) {
        config.enable_gravity_compensation = root["enable_gravity_compensation"].as<bool>();
    }
    if (root["visualization_enabled"]) {
        config.visualization_enabled = root["visualization_enabled"].as<bool>();
    }
    if (root["tracked_frame_names"]) {
        config.tracked_frame_names = root["tracked_frame_names"].as<std::vector<std::string>>();
    }

    config.ft_sensor_frame_ids.clear();
    config.ft_force_sensor_names.clear();
    config.ft_torque_sensor_names.clear();

    if (root["ft_sensors"]) {
        const YAML::Node ft_sensors = root["ft_sensors"];
        if (!ft_sensors.IsSequence()) {
            throw std::runtime_error("'ft_sensors' must be a sequence in config file: " + config_path);
        }

        for (const auto& ft_sensor : ft_sensors) {
            if (!ft_sensor["frame_id"] || !ft_sensor["force_sensor_name"] || !ft_sensor["torque_sensor_name"]) {
                throw std::runtime_error(
                    "Each 'ft_sensors' entry must contain frame_id, force_sensor_name, and torque_sensor_name.");
            }
            config.ft_sensor_frame_ids.push_back(ft_sensor["frame_id"].as<std::string>());
            config.ft_force_sensor_names.push_back(ft_sensor["force_sensor_name"].as<std::string>());
            config.ft_torque_sensor_names.push_back(ft_sensor["torque_sensor_name"].as<std::string>());
        }
    }
}

std::string resolve_plugin_directory() {
    const char* conda_prefix = std::getenv("CONDA_PREFIX");
    const char* mujoco_path = std::getenv("MUJOCO_PATH");

    if (conda_prefix && std::strlen(conda_prefix) > 0) {
        return std::string(conda_prefix) + "/bin/mujoco_plugin";
    }
    if (mujoco_path && std::strlen(mujoco_path) > 0) {
        return std::string(mujoco_path) + "/bin/mujoco_plugin";
    }

    throw std::runtime_error("Missing --plugin-dir and could not infer one from CONDA_PREFIX or MUJOCO_PATH.");
}

void validate_paths(const MujocoSimCore::Config& config) {
    namespace fs = std::filesystem;

    if (!fs::exists(config.xml_location)) {
        throw std::runtime_error("Model XML does not exist: " + config.xml_location);
    }
    if (!fs::exists(config.plugin_directory)) {
        throw std::runtime_error("MuJoCo plugin directory does not exist: " + config.plugin_directory);
    }
}

void print_startup_summary(const MujocoSimCore::Config& config, const MujocoSimCore& sim_core) {
    std::cout << "Standalone MuJoCo viewer configuration\n";
    std::cout << "  XML: " << config.xml_location << "\n";
    std::cout << "  Plugin dir: " << config.plugin_directory << "\n";
    std::cout << "  Control mode: " << control_mode_label(config.control_mode) << "\n";
    std::cout << "  Simulation frequency: " << config.simulation_frequency << " Hz\n";
    std::cout << "  Gravity compensation: " << (config.enable_gravity_compensation ? "enabled" : "disabled") << "\n";
    std::cout << "  Joint damping gain: " << config.joint_damping_gain << "\n";
    std::cout << "  Visualization: " << (config.visualization_enabled ? "enabled" : "disabled") << "\n";
    std::cout << "  Fixed cameras: " << (sim_core.has_fixed_cameras() ? "yes" : "no") << "\n";
    std::cout << "  Tracked frames: " << sim_core.tracked_frame_configs().size() << "\n";
    for (const auto& frame : sim_core.tracked_frame_configs()) {
        std::cout << "    - " << frame.name << " ("
                  << (frame.type == MujocoSimCore::TrackedFrameType::BODY ? "body" : "site") << ")\n";
    }
    std::cout << "  FT outputs: " << sim_core.ft_sensor_configs().size() << "\n";
    for (const auto& ft_sensor : sim_core.ft_sensor_configs()) {
        std::cout << "    - " << ft_sensor.frame_id << " [force=" << ft_sensor.force_sensor_name
                  << ", torque=" << ft_sensor.torque_sensor_name << "]\n";
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    MujocoSimCore::Config config;
    config.control_mode = POSITION;
    config.enable_gravity_compensation = false;
    config.simulation_frequency = 1000;
    config.joint_damping_gain = 0.0;
    config.visualization_enabled = true;
    std::string standalone_config_path;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) {
                standalone_config_path = argv[++i];
                load_config_file(standalone_config_path, config);
            } else if (arg == "--xml" && i + 1 < argc) {
                config.xml_location = argv[++i];
            } else if (arg == "--plugin-dir" && i + 1 < argc) {
                config.plugin_directory = argv[++i];
            } else if (arg == "--mode" && i + 1 < argc) {
                config.control_mode = parse_control_mode(argv[++i]);
            } else if (arg == "--sim-frequency" && i + 1 < argc) {
                config.simulation_frequency = parse_positive_int(argv[++i], "--sim-frequency");
            } else if (arg == "--joint-damping-gain" && i + 1 < argc) {
                config.joint_damping_gain = parse_double(argv[++i], "--joint-damping-gain");
            } else if (arg == "--visualization-enabled" && i + 1 < argc) {
                config.visualization_enabled = parse_bool(argv[++i], "--visualization-enabled");
            } else if (arg == "--enable-gravity-compensation") {
                config.enable_gravity_compensation = true;
            } else if (arg == "--tracked-frame" && i + 1 < argc) {
                config.tracked_frame_names.push_back(argv[++i]);
            } else if (arg == "--ft" && i + 1 < argc) {
                parse_force_torque_spec(argv[++i], config);
            } else if (arg == "--help" || arg == "-h") {
                print_usage();
                return 0;
            } else {
                throw std::runtime_error("Unknown or incomplete argument: " + arg);
            }
        }

        if (config.xml_location.empty()) {
            throw std::runtime_error("Missing model XML. Provide --xml <model.xml> or set xml_location in --config.");
        }

        if (config.plugin_directory.empty()) {
            config.plugin_directory = resolve_plugin_directory();
        }

        validate_paths(config);

        MujocoSimCore sim_core(config);
        print_startup_summary(config, sim_core);

        std::atomic<bool> paused{false};
        std::atomic<bool> reset_requested{false};
        std::atomic<bool> simulation_paused{false};
        std::atomic<bool> simulation_reset_requested{false};
        std::atomic<bool> simulation_reset_completed{false};
        std::vector<std::atomic<double>> pending_jog_values(sim_core.model()->nu);
        std::vector<std::atomic<bool>> pending_jog_dirty(sim_core.model()->nu);
        for (std::size_t actuator_id = 0; actuator_id < pending_jog_values.size(); ++actuator_id) {
            pending_jog_values[actuator_id].store(0.0, std::memory_order_release);
            pending_jog_dirty[actuator_id].store(false, std::memory_order_release);
        }

        MujocoViewer viewer;
        if (config.visualization_enabled) {
            viewer.initialize(*sim_core.model(), false, &paused, &reset_requested);
            viewer.initialize_control_panel(
                config.control_mode,
                [&pending_jog_values, &pending_jog_dirty](std::size_t actuator_id, double value) {
                    if (actuator_id >= pending_jog_values.size()) {
                        return;
                    }
                    pending_jog_values[actuator_id].store(value, std::memory_order_release);
                    pending_jog_dirty[actuator_id].store(true, std::memory_order_release);
                },
                true);
            viewer.set_camera_properties({0.0, 0.0, 0.5}, 2.5, 135.0, -30.0, false);
        }

        if (!config.visualization_enabled) {
            while (true) {
                sim_core.step();
            }
        }

        std::atomic<bool> run_simulation{true};
        std::unique_ptr<mjData, void (*)(mjData*)> render_data(mj_makeData(sim_core.model()), mj_deleteData);
        if (!render_data) {
            throw std::runtime_error("Failed to allocate render mjData snapshot.");
        }
        const auto step_period = std::chrono::duration<double>(1.0 / static_cast<double>(config.simulation_frequency));
        std::thread simulation_thread([&]() {
            using clock_type = std::chrono::steady_clock;
            auto next_tick = clock_type::now();

            while (run_simulation.load()) {
                next_tick += std::chrono::duration_cast<clock_type::duration>(step_period);

                if (simulation_reset_requested.exchange(false)) {
                    sim_core.reset();
                    for (std::size_t actuator_id = 0; actuator_id < pending_jog_values.size(); ++actuator_id) {
                        pending_jog_values[actuator_id].store(0.0, std::memory_order_release);
                        pending_jog_dirty[actuator_id].store(false, std::memory_order_release);
                    }
                    simulation_reset_completed.store(true, std::memory_order_release);
                }

                for (std::size_t actuator_id = 0; actuator_id < pending_jog_values.size(); ++actuator_id) {
                    if (!pending_jog_dirty[actuator_id].exchange(false, std::memory_order_acq_rel)) {
                        continue;
                    }

                    switch (config.control_mode) {
                        case POSITION:
                            sim_core.set_position_command(
                                actuator_id, pending_jog_values[actuator_id].load(std::memory_order_acquire));
                            break;
                        case TORQUE:
                            sim_core.set_effort_command(
                                actuator_id, pending_jog_values[actuator_id].load(std::memory_order_acquire));
                            break;
                        default:
                            break;
                    }
                }

                if (!simulation_paused.load()) {
                    sim_core.step();
                }

                std::this_thread::sleep_until(next_tick);
                const auto now = clock_type::now();
                if (now > next_tick + std::chrono::duration_cast<clock_type::duration>(step_period)) {
                    next_tick = now;
                }
            }
        });

        try {
            while (true) {
                simulation_paused.store(paused.load(std::memory_order_acquire), std::memory_order_release);
                if (reset_requested.exchange(false, std::memory_order_acq_rel)) {
                    simulation_reset_requested.store(true, std::memory_order_release);
                }
                if (simulation_reset_completed.exchange(false, std::memory_order_acq_rel)) {
                    viewer.invalidate_control_panel_values();
                }

                ViewerStatus viewer_status;
                viewer_status.control_mode = control_mode_label(config.control_mode);
                viewer_status.paused = paused.load(std::memory_order_acquire);
                viewer_status.collision_visible = viewer.collision_visible();
                viewer_status.contacts_visible = viewer.contacts_visible();
                viewer_status.control_panel_available = viewer.control_panel_available();
                viewer_status.control_panel_visible = viewer.control_panel_visible();
                viewer_status.jog_enabled = viewer.jog_enabled();
                viewer_status.control_slider_count = static_cast<int>(viewer.control_slider_count());
                viewer_status.simulation_frequency = config.simulation_frequency;
                viewer_status.contact_force_scale = viewer.contact_force_scale();
                viewer_status.fixed_cameras_available = sim_core.has_fixed_cameras();

                {
                    std::lock_guard<std::recursive_mutex> lock(sim_core.state_mutex());
                    mj_copyData(render_data.get(), sim_core.model(), sim_core.data());
                }

                viewer_status.contact_count = render_data->ncon;
                viewer_status.ft_available = false;
                for (const auto& ft_state : sim_core.force_torque_states(*render_data)) {
                    viewer_status.ft_available = viewer_status.ft_available || ft_state.available;
                    viewer_status.force_torque_sensors.push_back({ft_state.frame_id, ft_state.force, ft_state.torque});
                }

                for (const auto& tracked_frame : sim_core.tracked_frame_states(*render_data)) {
                    viewer_status.tracked_frames.push_back({tracked_frame.name, tracked_frame.position});
                }

                viewer.set_status(viewer_status);
                viewer.update_scene(*render_data);
                viewer.present();
            }
        } catch (...) {
            run_simulation = false;
            if (simulation_thread.joinable()) {
                simulation_thread.join();
            }
            throw;
        }

        run_simulation = false;
        if (simulation_thread.joinable()) {
            simulation_thread.join();
        }
    } catch (const std::exception& error) {
        std::cerr << "mujoco_sim_viewer failed: " << error.what() << "\n\n";
        print_usage();
        return 1;
    }
}
