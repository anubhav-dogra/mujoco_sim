#include <mujoco_viewer/MujocoViewer.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <opencv4/opencv2/opencv.hpp>
#include <stdexcept>
#include <utility>

MujocoViewer::~MujocoViewer() {
    if (_model) {
        mjv_freeScene(&_scene);
        mjr_freeContext(&_context);
    }
    if (_window) {
        glfwDestroyWindow(_window);
    }
    if (_model || _window) {
        glfwTerminate();
    }
}

void MujocoViewer::initialize(mjModel& model, bool large_hud, std::atomic<bool>* paused,
                              std::atomic<bool>* reset_requested) {
    _model = &model;
    _paused = paused;
    _reset_requested = reset_requested;
    _viewer_controls.set_hud_enabled(true);
    _viewer_controls.set_hud_large(large_hud);

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }

    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);
    if (!_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window.");
    }

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_rendering_options);
    _viewer_controls.initialize(model, _rendering_options);
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(&model, &_scene, 2000);
    mjr_makeContext(&model, &_context, mjFONTSCALE_100);
    _control_ui.spacing = mjui_themeSpacing(0);
    _control_ui.color = mjui_themeColor(0);
    _control_ui.rectid = 1;
    _control_ui.auxid = 0;
    _control_ui.radiocol = 1;
    _ui_state.nrect = 2;
    _ui_state.userdata = this;
    mjr_addAux(_control_ui.auxid, 320, 900, _control_ui.spacing.samples, &_context);
    update_ui_layout();

    glfwSetWindowUserPointer(_window, this);
    glfwSetKeyCallback(_window, MujocoViewer::keyboard_callback);
    glfwSetCursorPosCallback(_window, MujocoViewer::mouse_move_callback);
    glfwSetMouseButtonCallback(_window, MujocoViewer::mouse_button_callback);
    glfwSetScrollCallback(_window, MujocoViewer::scroll_callback);
}

bool MujocoViewer::is_initialized() const { return _window != nullptr; }

bool MujocoViewer::should_close() const { return _window && glfwWindowShouldClose(_window); }

void MujocoViewer::initialize_control_panel(ControlMode mode, ControlCommandCallback callback, bool visible) {
    _control_mode = mode;
    _control_callback = std::move(callback);
    _show_control_panel = visible;
    _jog_enabled = true;
    _control_values_initialized = false;
    rebuild_control_panel();
}

void MujocoViewer::set_camera_properties(const std::array<double, 3>& focal_point, double distance, double azimuth,
                                         double elevation, bool orthographic) {
    _camera.lookat[0] = focal_point[0];
    _camera.lookat[1] = focal_point[1];
    _camera.lookat[2] = focal_point[2];
    _camera.distance = distance;
    _camera.azimuth = azimuth;
    _camera.elevation = elevation;
    _camera.orthographic = orthographic;
}

void MujocoViewer::set_status(ViewerStatus status) {
    _viewer_status = std::move(status);
    _viewer_controls.set_status(_viewer_status);
}

bool MujocoViewer::collision_visible() const { return _rendering_options.geomgroup[3] != 0; }

bool MujocoViewer::contacts_visible() const {
    return _rendering_options.flags[mjVIS_CONTACTPOINT] || _rendering_options.flags[mjVIS_CONTACTFORCE];
}

float MujocoViewer::contact_force_scale() const { return _viewer_controls.contact_force_scale(); }

bool MujocoViewer::control_panel_available() const { return !_control_actuator_ids.empty(); }

bool MujocoViewer::control_panel_visible() const { return _show_control_panel && control_panel_available(); }

bool MujocoViewer::jog_enabled() const { return _jog_enabled; }

std::size_t MujocoViewer::control_slider_count() const { return _control_actuator_ids.size(); }

void MujocoViewer::invalidate_control_panel_values() { _control_values_initialized = false; }

void MujocoViewer::update_scene(mjData& joint_state) {
    if (!_model || !_window) {
        return;
    }

    glfwMakeContextCurrent(_window);
    update_ui_layout();
    if (control_panel_available() && !_control_values_initialized && _ui_state.dragrect != _control_ui.rectid) {
        for (std::size_t i = 0; i < _control_actuator_ids.size() && i < _control_values.size(); ++i) {
            _control_values[i] = joint_state.ctrl[_control_actuator_ids[i]];
        }
        _control_values_initialized = true;
        mjui_update(-1, -1, &_control_ui, &_ui_state, &_context);
    }
    mjv_updateScene(_model, &joint_state, &_rendering_options, nullptr, &_camera, mjCAT_ALL, &_scene);
}

void MujocoViewer::present() {
    if (!_model || !_window) {
        return;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(_window, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    mjr_render(viewport, &_scene, &_context);
    if (control_panel_visible()) {
        mjui_render(&_control_ui, &_ui_state, &_context);
    }
    _viewer_controls.render_overlay(viewport, _context);
    glfwSwapBuffers(_window);
    glfwPollEvents();
}

void MujocoViewer::render(mjData& joint_state) {
    update_scene(joint_state);
    present();
}

bool MujocoViewer::render_fixed_camera_image(mjData& joint_state, int fixed_camera_id, const std::string& frame_name,
                                             RenderedImage& image) {
    if (!_model || !_window || fixed_camera_id < 0 || fixed_camera_id >= _model->ncam) {
        return false;
    }

    mjvCamera eye_camera;
    mjv_defaultCamera(&eye_camera);
    eye_camera.type = mjCAMERA_FIXED;
    eye_camera.fixedcamid = fixed_camera_id;

    glfwMakeContextCurrent(_window);
    mjv_updateScene(_model, &joint_state, &_rendering_options, nullptr, &eye_camera, mjCAT_ALL, &_scene);

    int cam_width = 1280;
    int cam_height = 720;
    mjrRect viewport = {0, 0, cam_width, cam_height};

    mjr_render(viewport, &_scene, &_context);

    image.data.resize(cam_width * cam_height * 3);
    mjr_readPixels(image.data.data(), nullptr, viewport, &_context);
    image.frame_name = frame_name;
    image.height = cam_height;
    image.width = cam_width;

    cv::Mat img(cv::Size(cam_width, cam_height), CV_8UC3, image.data.data());
    cv::Mat flipped_img;
    cv::flip(img, flipped_img, 1);
    std::memcpy(image.data.data(), flipped_img.data, image.data.size());

    return true;
}

void MujocoViewer::scroll_callback(GLFWwindow* window, double /*xoffset*/, double yoffset) {
    auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->scroll(yoffset);
    }
}

void MujocoViewer::keyboard_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->keyboard(key, action);
    }
}

void MujocoViewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    (void)mods;
    auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->mouse_button(button, action);
    }
}

void MujocoViewer::mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* viewer = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        viewer->mouse_move(xpos, ypos);
    }
}

void MujocoViewer::scroll(double yoffset) {
    if (handle_ui_event(mjEVENT_SCROLL, current_cursor_x(), current_cursor_y(), mjBUTTON_NONE, 0, 0.0, yoffset)) {
        return;
    }
    mjv_moveCamera(_model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &_scene, &_camera);
}

void MujocoViewer::keyboard(int key, int action) {
    if (!_paused || !_reset_requested) {
        return;
    }
    bool paused = _paused->load(std::memory_order_acquire);
    bool reset_requested = _reset_requested->load(std::memory_order_acquire);
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_U && control_panel_available()) {
            _show_control_panel = !_show_control_panel;
            update_ui_layout();
        } else if (key == GLFW_KEY_G && control_panel_available()) {
            _jog_enabled = !_jog_enabled;
        }
    }
    _viewer_controls.handle_key(key, action, paused, reset_requested);
    _paused->store(paused, std::memory_order_release);
    if (reset_requested) {
        _reset_requested->store(true, std::memory_order_release);
    }
}

void MujocoViewer::mouse_button(int button, int action) {
    const double cursor_x = current_cursor_x();
    const double cursor_y = current_cursor_y();
    int ui_button = mjBUTTON_NONE;
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
            ui_button = mjBUTTON_LEFT;
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            ui_button = mjBUTTON_RIGHT;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            ui_button = mjBUTTON_MIDDLE;
            break;
        default:
            break;
    }

    if (handle_ui_event(action == GLFW_PRESS ? mjEVENT_PRESS : mjEVENT_RELEASE, cursor_x, cursor_y, ui_button)) {
        if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_MIDDLE ||
            button == GLFW_MOUSE_BUTTON_RIGHT) {
            _button_left = false;
            _button_middle = false;
            _button_right = false;
        }
        _last_x = cursor_x;
        _last_y = cursor_y;
        return;
    }

    _button_left = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    _button_middle = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    _button_right = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(_window, &_last_x, &_last_y);
}

void MujocoViewer::mouse_move(double xpos, double ypos) {
    int ui_button = mjBUTTON_NONE;
    if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        ui_button = mjBUTTON_LEFT;
    } else if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        ui_button = mjBUTTON_RIGHT;
    } else if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        ui_button = mjBUTTON_MIDDLE;
    }

    if (handle_ui_event(mjEVENT_MOVE, xpos, ypos, ui_button)) {
        _last_x = xpos;
        _last_y = ypos;
        return;
    }

    if (!_button_left && !_button_middle && !_button_right) {
        return;
    }

    const double dx = xpos - _last_x;
    const double dy = ypos - _last_y;
    _last_x = xpos;
    _last_y = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(_window, &width, &height);

    const bool mod_shift = (glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (_button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (_button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(_model, action, dx / height, dy / height, &_scene, &_camera);
}

bool MujocoViewer::update_ui_layout() {
    if (!_window) {
        return false;
    }

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(_window, &framebuffer_width, &framebuffer_height);
    _ui_state.rect[0] = {0, 0, framebuffer_width, framebuffer_height};

    if (!control_panel_available() || !_show_control_panel) {
        _ui_state.rect[_control_ui.rectid] = {framebuffer_width, 0, 0, framebuffer_height};
        return false;
    }

    mjui_resize(&_control_ui, &_context);
    const int panel_width = _control_ui.width > 0 ? _control_ui.width : 320;
    _ui_state.rect[_control_ui.rectid] = {std::max(0, framebuffer_width - panel_width), 0,
                                          std::min(panel_width, framebuffer_width), framebuffer_height};
    const int target_width = std::max(1, _ui_state.rect[_control_ui.rectid].width);
    const int target_height = std::max(1, _ui_state.rect[_control_ui.rectid].height);
    const bool resized = target_width != _control_panel_width || target_height != _control_panel_height;
    if (resized) {
        _control_panel_width = target_width;
        _control_panel_height = target_height;
        mjr_addAux(_control_ui.auxid, _control_panel_width, _control_panel_height, _control_ui.spacing.samples,
                   &_context);
    }
    mjui_resize(&_control_ui, &_context);
    if (resized) {
        mjui_update(-1, -1, &_control_ui, &_ui_state, &_context);
    }
    return true;
}

bool MujocoViewer::is_cursor_in_control_panel(double xpos, double ypos) const {
    if (!_window || !control_panel_visible()) {
        return false;
    }

    int window_width = 0;
    int window_height = 0;
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetWindowSize(_window, &window_width, &window_height);
    glfwGetFramebufferSize(_window, &framebuffer_width, &framebuffer_height);
    if (window_width <= 0 || window_height <= 0) {
        return false;
    }

    const double scale_x = static_cast<double>(framebuffer_width) / static_cast<double>(window_width);
    const double scale_y = static_cast<double>(framebuffer_height) / static_cast<double>(window_height);
    const int x = static_cast<int>(xpos * scale_x);
    const int y = static_cast<int>((window_height - ypos) * scale_y);
    const mjrRect& panel_rect = _ui_state.rect[_control_ui.rectid];
    return x >= panel_rect.left && x < panel_rect.left + panel_rect.width && y >= panel_rect.bottom &&
           y < panel_rect.bottom + panel_rect.height;
}

bool MujocoViewer::handle_ui_event(mjtEvent type, double xpos, double ypos, int button, int key, double sx, double sy) {
    if (!_window || !control_panel_available()) {
        return false;
    }

    update_ui_layout();
    const bool cursor_in_panel = is_cursor_in_control_panel(xpos, ypos);
    const bool ui_drag_active = _ui_state.dragrect == _control_ui.rectid;

    if (!_show_control_panel || (!cursor_in_panel && !ui_drag_active && type != mjEVENT_KEY)) {
        return false;
    }

    int window_width = 0;
    int window_height = 0;
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetWindowSize(_window, &window_width, &window_height);
    glfwGetFramebufferSize(_window, &framebuffer_width, &framebuffer_height);
    if (window_width <= 0 || window_height <= 0) {
        return false;
    }

    const double scale_x = static_cast<double>(framebuffer_width) / static_cast<double>(window_width);
    const double scale_y = static_cast<double>(framebuffer_height) / static_cast<double>(window_height);

    _ui_state.type = type;
    _ui_state.left = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    _ui_state.right = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    _ui_state.middle = glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    _ui_state.control = glfwGetKey(_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(_window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
    _ui_state.shift = glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    _ui_state.alt =
        glfwGetKey(_window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS || glfwGetKey(_window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
    _ui_state.button = button;
    _ui_state.key = key;
    _ui_state.x = xpos * scale_x;
    _ui_state.y = (window_height - ypos) * scale_y;
    _ui_state.dx = (_ui_state.x - _last_x * scale_x);
    _ui_state.dy = (_ui_state.y - ((window_height - _last_y) * scale_y));
    _ui_state.sx = sx;
    _ui_state.sy = sy;

    mjuiItem* changed_item = mjui_event(&_control_ui, &_ui_state, &_context);
    if (changed_item) {
        handle_control_item_change(*changed_item);
    }
    return cursor_in_panel || ui_drag_active || changed_item != nullptr;
}

void MujocoViewer::rebuild_control_panel() {
    _control_values.clear();
    _control_actuator_ids.clear();
    _control_values_initialized = false;
    std::memset(&_control_ui, 0, sizeof(_control_ui));
    _control_ui.spacing = mjui_themeSpacing(0);
    _control_ui.color = mjui_themeColor(0);
    _control_ui.rectid = 1;
    _control_ui.auxid = 0;
    _control_ui.radiocol = 1;

    if (!_model || !_control_callback || _control_mode == VELOCITY) {
        return;
    }

    _control_values.reserve(_model->nu);
    _control_actuator_ids.reserve(_model->nu);
    std::vector<mjuiDef> definitions;
    definitions.reserve(static_cast<std::size_t>(_model->nu) + 2);

    mjuiDef section_def{};
    section_def.type = mjITEM_SECTION;
    std::snprintf(section_def.name, sizeof(section_def.name), "Jog Controls");
    section_def.state = mjSECT_OPEN;
    definitions.push_back(section_def);

    for (int actuator_id = 0; actuator_id < _model->nu; ++actuator_id) {
        _control_actuator_ids.push_back(actuator_id);
        _control_values.push_back(0.0);

        mjuiDef slider_def{};
        slider_def.type = mjITEM_SLIDERNUM;
        const std::string label = control_item_label(actuator_id);
        std::snprintf(slider_def.name, sizeof(slider_def.name), "%s", label.c_str());
        slider_def.state = 1;
        slider_def.pdata = &_control_values.back();

        const double range_min =
            _model->actuator_ctrllimited[actuator_id] ? _model->actuator_ctrlrange[2 * actuator_id] : -1.0;
        const double range_max =
            _model->actuator_ctrllimited[actuator_id] ? _model->actuator_ctrlrange[2 * actuator_id + 1] : 1.0;
        std::snprintf(slider_def.other, sizeof(slider_def.other), "%.6g %.6g", range_min, range_max);
        definitions.push_back(slider_def);
    }

    mjuiDef end_def{};
    end_def.type = mjITEM_END;
    definitions.push_back(end_def);
    mjui_add(&_control_ui, definitions.data());

    int slider_index = 0;
    for (int section_index = 0; section_index < _control_ui.nsect; ++section_index) {
        auto& section = _control_ui.sect[section_index];
        for (int item_index = 0; item_index < section.nitem; ++item_index) {
            auto& item = section.item[item_index];
            if (item.type == mjITEM_SLIDERNUM && slider_index < static_cast<int>(_control_actuator_ids.size())) {
                item.userid = _control_actuator_ids[static_cast<std::size_t>(slider_index)];
                ++slider_index;
            }
        }
    }

    update_ui_layout();
    mjui_update(-1, -1, &_control_ui, &_ui_state, &_context);
}

void MujocoViewer::handle_control_item_change(const mjuiItem& item) {
    if (!_control_callback || !_jog_enabled || item.type != mjITEM_SLIDERNUM) {
        return;
    }

    const auto* value = static_cast<const mjtNum*>(item.pdata);
    if (!value) {
        return;
    }

    _control_callback(static_cast<std::size_t>(item.userid), static_cast<double>(*value));
}

std::string MujocoViewer::control_item_label(int actuator_id) const {
    if (!_model || actuator_id < 0 || actuator_id >= _model->nu) {
        return "control";
    }

    const char* actuator_name = _model->names + _model->name_actuatoradr[actuator_id];
    if (_model->actuator_trntype[actuator_id] == mjTRN_JOINT ||
        _model->actuator_trntype[actuator_id] == mjTRN_JOINTINPARENT) {
        const int joint_id = _model->actuator_trnid[2 * actuator_id];
        if (joint_id >= 0 && joint_id < _model->njnt) {
            return _model->names + _model->name_jntadr[joint_id];
        }
    }
    return actuator_name;
}

double MujocoViewer::current_cursor_x() const {
    double xpos = 0.0;
    double ypos = 0.0;
    if (_window) {
        glfwGetCursorPos(_window, &xpos, &ypos);
    }
    return xpos;
}

double MujocoViewer::current_cursor_y() const {
    double xpos = 0.0;
    double ypos = 0.0;
    if (_window) {
        glfwGetCursorPos(_window, &xpos, &ypos);
    }
    return ypos;
}
