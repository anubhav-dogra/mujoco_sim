#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <mujoco_core/MujocoSimCore.h>
#include <mujoco_viewer/ViewerControls.h>

// Owns the MuJoCo viewer window, render state, and camera image rendering.
// ViewerControls remains separate and manages HUD state and viewer toggles.
class MujocoViewer {
   public:
    using ControlCommandCallback = std::function<void(std::size_t actuator_id, double value)>;

    struct RenderedImage {
        std::string frame_name;
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> data;
    };

    MujocoViewer() = default;
    ~MujocoViewer();

    void initialize(mjModel &model, bool large_hud, std::atomic<bool> *paused, std::atomic<bool> *reset_requested);
    bool is_initialized() const;
    void initialize_control_panel(ControlMode mode, ControlCommandCallback callback, bool visible = true);

    void set_camera_properties(const std::array<double, 3> &focal_point, double distance, double azimuth,
                               double elevation, bool orthographic);
    void set_status(ViewerStatus status);

    bool collision_visible() const;
    bool contacts_visible() const;
    float contact_force_scale() const;
    bool control_panel_available() const;
    bool control_panel_visible() const;
    bool jog_enabled() const;
    std::size_t control_slider_count() const;
    void invalidate_control_panel_values();

    void update_scene(mjData &joint_state);
    void present();
    void render(mjData &joint_state);
    bool render_fixed_camera_image(mjData &joint_state, int fixed_camera_id, const std::string &frame_name,
                                   RenderedImage &image);

   private:
    static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
    static void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
    static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);

    void scroll(double yoffset);
    void keyboard(int key, int action);
    void mouse_button(int button, int action);
    void mouse_move(double xpos, double ypos);
    bool update_ui_layout();
    bool is_cursor_in_control_panel(double xpos, double ypos) const;
    bool handle_ui_event(mjtEvent type, double xpos, double ypos, int button = mjBUTTON_NONE, int key = 0,
                         double sx = 0.0, double sy = 0.0);
    void rebuild_control_panel();
    void handle_control_item_change(const mjuiItem &item);
    std::string control_item_label(int actuator_id) const;
    double current_cursor_x() const;
    double current_cursor_y() const;

    mjModel *_model = nullptr;
    mjvCamera _camera;
    mjvOption _rendering_options;
    mjvPerturb _perturbation;
    mjvScene _scene;
    mjrContext _context;
    mjUI _control_ui{};
    mjuiState _ui_state{};
    GLFWwindow *_window = nullptr;
    std::atomic<bool> *_paused = nullptr;
    std::atomic<bool> *_reset_requested = nullptr;
    bool _button_left = false;
    bool _button_middle = false;
    bool _button_right = false;
    double _last_x = 0.0;
    double _last_y = 0.0;
    bool _show_control_panel = false;
    bool _jog_enabled = true;
    bool _control_values_initialized = false;
    int _control_panel_width = 0;
    int _control_panel_height = 0;
    ControlMode _control_mode = UNKNOWN;
    ControlCommandCallback _control_callback;
    std::vector<mjtNum> _control_values;
    std::vector<int> _control_actuator_ids;
    ViewerControls _viewer_controls;
    ViewerStatus _viewer_status;
};
