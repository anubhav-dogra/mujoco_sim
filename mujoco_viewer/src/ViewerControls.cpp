#include <mujoco_viewer/ViewerControls.h>

#include <algorithm>
#include <cstring>

void ViewerControls::initialize(mjModel &model, mjvOption &rendering_options) {
    _model = &model;
    _rendering_options = &rendering_options;
    for (int i = 0; i < mjNGROUP; ++i) {
        rendering_options.sitegroup[i] = 1;
    }
    rendering_options.geomgroup[3] = 0;
    rendering_options.flags[mjVIS_CONTACTPOINT] = 0;
    rendering_options.flags[mjVIS_CONTACTFORCE] = 0;
    rendering_options.frame = mjFRAME_NONE;
    rendering_options.label = mjLABEL_NONE;

    _base_joint_length = model.vis.scale.jointlength;
    _base_joint_width = model.vis.scale.jointwidth;
    _base_frame_length = model.vis.scale.framelength;
    _base_frame_width = model.vis.scale.framewidth;
    _geom_base_alpha.assign(model.ngeom, 1.0f);
    for (int i = 0; i < model.ngeom; ++i) {
        _geom_base_alpha[i] = model.geom_rgba[4 * i + 3];
    }
    initialize_ft_figure();
}

void ViewerControls::handle_key(int key, int action, bool &paused, bool &reset_requested) {
    if (action != GLFW_PRESS) {
        return;
    }

    switch (key) {
        case GLFW_KEY_C:
            _show_collision_geoms = !_show_collision_geoms;
            if (_rendering_options) {
                _rendering_options->geomgroup[3] = _show_collision_geoms ? 1 : 0;
            }
            break;
        case GLFW_KEY_J:
            _show_joint_frames = !_show_joint_frames;
            if (_rendering_options) {
                _rendering_options->flags[mjVIS_JOINT] = _show_joint_frames;
            }
            break;
        case GLFW_KEY_M:
            if (_rendering_options) {
                _rendering_options->flags[mjVIS_COM] = !_rendering_options->flags[mjVIS_COM];
            }
            break;
        case GLFW_KEY_B:
            _frame_mode = _frame_mode == FrameMode::BODY ? FrameMode::NONE : FrameMode::BODY;
            if (_rendering_options) {
                _rendering_options->frame = _frame_mode == FrameMode::BODY ? mjFRAME_BODY : mjFRAME_NONE;
            }
            break;
        case GLFW_KEY_S:
            _frame_mode = _frame_mode == FrameMode::SITE ? FrameMode::NONE : FrameMode::SITE;
            if (_rendering_options) {
                _rendering_options->frame = _frame_mode == FrameMode::SITE ? mjFRAME_SITE : mjFRAME_NONE;
            }
            break;
        case GLFW_KEY_F:
            _show_contacts = !_show_contacts;
            if (_rendering_options) {
                _rendering_options->flags[mjVIS_CONTACTPOINT] = _show_contacts;
                _rendering_options->flags[mjVIS_CONTACTFORCE] = _show_contacts;
            }
            break;
        case GLFW_KEY_EQUAL:
        case GLFW_KEY_KP_ADD:
            scale_active_visuals(1.25f);
            break;
        case GLFW_KEY_MINUS:
        case GLFW_KEY_KP_SUBTRACT:
            scale_active_visuals(1.0f / 1.25f);
            break;
        case GLFW_KEY_P:
            paused = !paused;
            break;
        case GLFW_KEY_BACKSPACE:
            reset_requested = true;
            break;
        case GLFW_KEY_V:
            cycle_model_alpha();
            break;
        case GLFW_KEY_H:
            _show_help_overlay = !_show_help_overlay;
            break;
        case GLFW_KEY_T:
            _show_ft_plots = !_show_ft_plots;
            break;
        case GLFW_KEY_N:
            if (!_status.force_torque_sensors.empty()) {
                _ft_plot_sensor_index = (_ft_plot_sensor_index + 1) % _status.force_torque_sensors.size();
                _ft_plot_name.clear();
                _ft_plot_time_history.clear();
                for (auto &history : _ft_history) {
                    history.clear();
                }
                update_ft_figure();
            }
            break;
        case GLFW_KEY_L:
            if (_rendering_options) {
                _rendering_options->label = _rendering_options->label == mjLABEL_BODY ? mjLABEL_NONE : mjLABEL_BODY;
            }
            break;
        default:
            break;
    }
}

void ViewerControls::set_status(ViewerStatus status) {
    _status = std::move(status);
    if (_ft_plot_sensor_index >= _status.force_torque_sensors.size()) {
        _ft_plot_sensor_index = 0;
    }
    update_ft_figure();
}

void ViewerControls::set_hud_enabled(bool enabled) { _hud_enabled = enabled; }

void ViewerControls::set_hud_large(bool large) { _hud_large = large; }

float ViewerControls::contact_force_scale() const { return _contact_force_scale; }

const char *ViewerControls::label_mode_name() const {
    if (!_rendering_options) {
        return "off";
    }

    switch (_rendering_options->label) {
        case mjLABEL_BODY:
            return "body";
        case mjLABEL_JOINT:
            return "joint";
        case mjLABEL_GEOM:
            return "geom";
        case mjLABEL_SITE:
            return "site";
        default:
            return "off";
    }
}

void ViewerControls::apply_visual_scale() {
    if (!_model) {
        return;
    }

    _model->vis.scale.jointlength = _base_joint_length * _joint_scale_multiplier;
    _model->vis.scale.jointwidth = _base_joint_width * _joint_scale_multiplier;
    _model->vis.scale.framelength = _base_frame_length * _joint_scale_multiplier;
    _model->vis.scale.framewidth = _base_frame_width * _joint_scale_multiplier;
}

void ViewerControls::scale_active_visuals(float factor) {
    if (_show_contacts) {
        _contact_force_scale *= factor;
        _contact_force_scale = std::clamp(_contact_force_scale, 0.1f, 100.0f);
        return;
    }

    if (_show_joint_frames || _frame_mode != FrameMode::NONE) {
        _joint_scale_multiplier *= factor;
        _joint_scale_multiplier = std::clamp(_joint_scale_multiplier, 0.25f, 12.0f);
        apply_visual_scale();
    }
}

void ViewerControls::cycle_model_alpha() {
    if (!_model || _geom_base_alpha.empty()) {
        return;
    }

    static constexpr std::array<float, 4> kAlphaLevels = {1.0f, 0.6f, 0.3f, 0.12f};
    _alpha_cycle_index = (_alpha_cycle_index + 1) % kAlphaLevels.size();
    const float alpha_scale = kAlphaLevels[_alpha_cycle_index];

    for (int i = 0; i < _model->ngeom; ++i) {
        if (_model->geom_bodyid[i] == 0) {
            _model->geom_rgba[4 * i + 3] = _geom_base_alpha[static_cast<std::size_t>(i)];
            continue;
        }
        _model->geom_rgba[4 * i + 3] =
            std::clamp(_geom_base_alpha[static_cast<std::size_t>(i)] * alpha_scale, 0.02f, 1.0f);
    }
}

void ViewerControls::initialize_ft_figure() {
    mjv_defaultFigure(&_ft_figure);
    _ft_figure.flg_extend = 0;
    _ft_figure.flg_barplot = 0;
    _ft_figure.flg_legend = 1;
    _ft_figure.figurergba[0] = 0.1f;
    _ft_figure.figurergba[1] = 0.1f;
    _ft_figure.figurergba[2] = 0.1f;
    _ft_figure.figurergba[3] = 0.85f;
    _ft_figure.panergba[0] = 0.0f;
    _ft_figure.panergba[1] = 0.0f;
    _ft_figure.panergba[2] = 0.0f;
    _ft_figure.panergba[3] = 0.65f;
    _ft_figure.gridsize[0] = 4;
    _ft_figure.gridsize[1] = 4;
    std::strncpy(_ft_figure.xlabel, "samples", sizeof(_ft_figure.xlabel) - 1);

    constexpr std::array<const char *, 6> kLineNames = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
    constexpr std::array<std::array<float, 3>, 6> kLineColors = {{{1.0f, 0.3f, 0.3f},
                                                                  {0.3f, 1.0f, 0.3f},
                                                                  {0.3f, 0.5f, 1.0f},
                                                                  {1.0f, 0.7f, 0.3f},
                                                                  {0.8f, 0.3f, 1.0f},
                                                                  {0.3f, 1.0f, 1.0f}}};

    for (int line = 0; line < 6; ++line) {
        std::strncpy(_ft_figure.linename[line], kLineNames[static_cast<std::size_t>(line)],
                     sizeof(_ft_figure.linename[line]) - 1);
        _ft_figure.linergb[line][0] = kLineColors[static_cast<std::size_t>(line)][0];
        _ft_figure.linergb[line][1] = kLineColors[static_cast<std::size_t>(line)][1];
        _ft_figure.linergb[line][2] = kLineColors[static_cast<std::size_t>(line)][2];
    }
}

void ViewerControls::append_ft_plot_sample(const ViewerForceTorqueStatus &ft_sensor) {
    if (_ft_plot_name.empty() || _ft_plot_name != ft_sensor.name) {
        _ft_plot_name = ft_sensor.name;
        _ft_plot_time_history.clear();
        for (auto &history : _ft_history) {
            history.clear();
        }
    }

    const float next_sample = _ft_plot_time_history.empty() ? 0.0f : _ft_plot_time_history.back() + 1.0f;
    _ft_plot_time_history.push_back(next_sample);
    _ft_history[0].push_back(static_cast<float>(ft_sensor.force[0]));
    _ft_history[1].push_back(static_cast<float>(ft_sensor.force[1]));
    _ft_history[2].push_back(static_cast<float>(ft_sensor.force[2]));
    _ft_history[3].push_back(static_cast<float>(ft_sensor.torque[0]));
    _ft_history[4].push_back(static_cast<float>(ft_sensor.torque[1]));
    _ft_history[5].push_back(static_cast<float>(ft_sensor.torque[2]));

    if (_ft_plot_time_history.size() > kForceTorquePlotHistory) {
        _ft_plot_time_history.erase(_ft_plot_time_history.begin());
        for (auto &history : _ft_history) {
            history.erase(history.begin());
        }
    }
}

void ViewerControls::update_ft_figure() {
    initialize_ft_figure();

    if (_status.force_torque_sensors.empty()) {
        _ft_plot_sensor_index = 0;
        _ft_plot_name.clear();
        _ft_plot_time_history.clear();
        for (auto &history : _ft_history) {
            history.clear();
        }
        std::strncpy(_ft_figure.title, "FT Plot: <none>", sizeof(_ft_figure.title) - 1);
        return;
    }

    append_ft_plot_sample(_status.force_torque_sensors[_ft_plot_sensor_index]);

    std::string title = "FT Plot: " + _ft_plot_name;
    std::strncpy(_ft_figure.title, title.c_str(), sizeof(_ft_figure.title) - 1);

    const int point_count = static_cast<int>(_ft_plot_time_history.size());
    float min_value = 0.0f;
    float max_value = 0.0f;

    for (int line = 0; line < 6; ++line) {
        _ft_figure.linepnt[line] = point_count;
        for (int point = 0; point < point_count; ++point) {
            const float x = _ft_plot_time_history[static_cast<std::size_t>(point)];
            const float y = _ft_history[static_cast<std::size_t>(line)][static_cast<std::size_t>(point)];
            _ft_figure.linedata[line][2 * point] = x;
            _ft_figure.linedata[line][2 * point + 1] = y;
            min_value = std::min(min_value, y);
            max_value = std::max(max_value, y);
        }
    }

    if (point_count > 0) {
        _ft_figure.range[0][0] = _ft_plot_time_history.front();
        _ft_figure.range[0][1] = _ft_plot_time_history.back() > _ft_plot_time_history.front()
                                     ? _ft_plot_time_history.back()
                                     : _ft_plot_time_history.front() + 1.0f;
        if (min_value == max_value) {
            min_value -= 1.0f;
            max_value += 1.0f;
        }
        _ft_figure.range[1][0] = min_value;
        _ft_figure.range[1][1] = max_value;
    }
}

void ViewerControls::render_ft_plots(const mjrRect &viewport, const mjrContext &context) const {
    if (!_show_ft_plots || _ft_plot_time_history.empty()) {
        return;
    }

    const int plot_width = std::max(320, viewport.width / 3);
    const int plot_height = std::max(220, viewport.height / 3);
    mjrRect plot_rect = {viewport.width - plot_width - 12, 12, plot_width, plot_height};
    mjr_figure(plot_rect, const_cast<mjvFigure *>(&_ft_figure), &context);
}

void ViewerControls::render_overlay(const mjrRect &viewport, const mjrContext &context) const {
    if (!_hud_enabled) {
        return;
    }

    std::string title = "Viewer status";
    std::string content = "Mode: ";
    content += _status.control_mode;
    content += "\nState: ";
    content += _status.paused ? "paused" : "running";
    content += "\nCollision geoms: ";
    content += _status.collision_visible ? "on" : "off";
    content += "\nLabels: ";
    content += label_mode_name();
    content += "\nContacts: ";
    content += _status.contacts_visible ? "on" : "off";
    content += "\nControl panel: ";
    if (!_status.control_panel_available) {
        content += "unavailable";
    } else {
        content += _status.control_panel_visible ? "visible" : "hidden";
    }
    content += "\nJog controls: ";
    content += _status.jog_enabled ? "enabled" : "disabled";
    content += "\nJog sliders: ";
    content += std::to_string(_status.control_slider_count);
    content += "\nContact count: ";
    content += std::to_string(_status.contact_count);
    content += "\nContact scale: ";
    content += std::to_string(_status.contact_force_scale);
    content += "\nSim frequency: ";
    content += std::to_string(_status.simulation_frequency);
    content += " Hz";
    content += "\nFT: ";
    content += _status.ft_available ? "available" : "unavailable";
    content += "\nFT sensors: ";
    content += std::to_string(_status.force_torque_sensors.size());
    if (!_status.force_torque_sensors.empty()) {
        content += "\nFT plot sensor: ";
        content += _status.force_torque_sensors[_ft_plot_sensor_index].name;
    }
    content += "\nFixed cameras: ";
    content += _status.fixed_cameras_available ? "available" : "unavailable";
    content += "\nTracked frames:";
    if (_status.tracked_frames.empty()) {
        content += " <none>";
    } else {
        for (const auto &tracked_frame : _status.tracked_frames) {
            content += "\n- ";
            content += tracked_frame.name;
            content += ": ";
            content += std::to_string(tracked_frame.position[0]) + ", " + std::to_string(tracked_frame.position[1]) +
                       ", " + std::to_string(tracked_frame.position[2]);
        }
    }

    mjr_overlay(_hud_large ? mjFONT_BIG : mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, title.c_str(), content.c_str(),
                &context);
    render_ft_plots(viewport, context);

    if (_show_help_overlay) {
        constexpr const char *help_title = "Viewer shortcuts";
        constexpr const char *help_content =
            "C: collision geoms\n"
            "J: joint frames\n"
            "M: centers of mass\n"
            "B: body frames\n"
            "S: site frames\n"
            "L: body names\n"
            "F: contacts\n"
            "T: FT plot\n"
            "N: next FT sensor\n"
            "U: toggle control panel\n"
            "G: toggle jog commands\n"
            "+/-: scale active contacts/frames\n"
            "V: cycle model alpha\n"
            "P: pause\n"
            "Backspace: reset\n"
            "H: hide help";
        mjr_overlay(_hud_large ? mjFONT_BIG : mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, help_title, help_content,
                    &context);
    }
}
