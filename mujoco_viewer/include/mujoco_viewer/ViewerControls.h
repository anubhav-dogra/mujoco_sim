#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

struct ViewerTrackedFrameStatus {
    std::string name;
    std::array<double, 3> position{};
};

struct ViewerForceTorqueStatus {
    std::string name;
    std::array<double, 3> force{};
    std::array<double, 3> torque{};
};

struct ViewerStatus {
    const char *control_mode = "UNKNOWN";
    bool paused = false;
    bool collision_visible = false;
    bool contacts_visible = false;
    bool control_panel_available = false;
    bool control_panel_visible = false;
    bool jog_enabled = false;
    int control_slider_count = 0;
    int simulation_frequency = 0;
    int contact_count = 0;
    float contact_force_scale = 1.0f;
    bool ft_available = false;
    bool fixed_cameras_available = false;
    std::vector<ViewerForceTorqueStatus> force_torque_sensors;
    std::vector<ViewerTrackedFrameStatus> tracked_frames;
};

class ViewerControls {
   public:
    enum class FrameMode { NONE, BODY, SITE };

    ViewerControls() = default;

    void initialize(mjModel &model, mjvOption &rendering_options);
    void handle_key(int key, int action, bool &paused, bool &reset_requested);
    void set_status(ViewerStatus status);
    void set_hud_enabled(bool enabled);
    void set_hud_large(bool large);
    float contact_force_scale() const;
    void render_overlay(const mjrRect &viewport, const mjrContext &context) const;

   private:
    static constexpr int kForceTorquePlotHistory = 200;

    void apply_visual_scale();
    void scale_active_visuals(float factor);
    void cycle_model_alpha();
    const char *label_mode_name() const;
    void initialize_ft_figure();
    void append_ft_plot_sample(const ViewerForceTorqueStatus &ft_sensor);
    void update_ft_figure();
    void render_ft_plots(const mjrRect &viewport, const mjrContext &context) const;

    mjModel *_model = nullptr;
    mjvOption *_rendering_options = nullptr;
    bool _show_collision_geoms = false;
    bool _show_joint_frames = false;
    bool _show_contacts = false;
    FrameMode _frame_mode = FrameMode::NONE;
    bool _show_help_overlay = false;
    bool _hud_enabled = true;
    bool _hud_large = true;
    float _contact_force_scale = 1.0f;
    float _joint_scale_multiplier = 1.0f;
    float _base_joint_length = 0.0f;
    float _base_joint_width = 0.0f;
    float _base_frame_length = 0.0f;
    float _base_frame_width = 0.0f;
    std::vector<float> _geom_base_alpha;
    bool _show_ft_plots = false;
    std::string _ft_plot_name;
    std::size_t _ft_plot_sensor_index = 0;
    std::vector<float> _ft_plot_time_history;
    std::array<std::vector<float>, 6> _ft_history;
    mjvFigure _ft_figure;
    std::size_t _alpha_cycle_index = 0;
    ViewerStatus _status;
};
