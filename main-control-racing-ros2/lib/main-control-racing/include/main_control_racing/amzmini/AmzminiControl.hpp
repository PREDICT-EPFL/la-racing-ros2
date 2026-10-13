#ifndef SRC_AMZMINICONTROL_HPP
#define SRC_AMZMINICONTROL_HPP

#include "main_control_common/ControlBase.hpp"
#include "main_control_common/ControlMath.hpp"
#include "main_control_common/ControlUtils.hpp"
#include "AmzminiTypes.hpp"

#include "utils_racing/Path.hpp"
#include "utils_racing/PathUtils.hpp"
#include "main_control_racing/amzmini/PurePursuitControl.hpp"

#include "utils_racing/LapTimer.hpp"

namespace control {
namespace racing {
namespace amzmini {

using namespace amzmini_types;

struct AmzminiProperties
{
    double wheelbase{9.8e-2}; // m
    double rear_axle_offset{0.051};
    double mass{144e-3}; // kg
    double steering_angle_max{radians(11)};
    double F_centri_max{9.81 * mass};
    double acc_max{15};
};

struct TestState
{
    const Vector2 track_pos{2, 2.1};
    const double track_angle{radians(80)};

    const Vector2 pos_global{3, 2};
    const double turn_angle_global{radians(10)};
    const Vector2 vel_global{1.0, 0.1};
    const double turn_rate{radians(5)};

    const Vector2 acc{0.3, 0.2};

    const Manual manual{0.4, 0.8, true};
};

class AmzminiControl : public virtual control_common::ControlBase
{
public:
    void load_track_from_yaml(const std::string &yaml_filepath)
    {
        m_track->load_from_yaml(yaml_filepath);
    }
protected:
    void init_impl() override;
    void test_impl() override;

    void compute_control_impl() override;

    /** Vehicle properties **/
    AmzminiProperties amzminiProperties{};

    /** States and controls **/
    unsigned p_state_notAvail_lim_ms{50};
    unsigned p_manual_notAvail_lim_ms{150};

    bool state_from_sim{false};
    TimePoint state_timestamp = SteadyClock::now();
    bool state_isAvail{false};
    Vector2 pos{0, 0}; // Position in track frame
    Vector2 vel{0, 0}; // Velocity in track frame
    Vector2 b_vel{0, 0}; // Velocity in body frame
    double turn_angle{0};
    double turn_rate{0};

    bool track_isAvail{false};
    Vector2 track_pos{0, 0};
    double track_angle{0};

    void update_local_states(const Vector2 &pos_global, const Vector2 &vel_global, double turn_angle_global);
    Eigen::Matrix2d T_bw;

    /* Manual control */
    Manual manual{};
    bool manual_isAvail{false};
    TimePoint manual_timestamp = SteadyClock::now();
    control_common::utils::ActiveParser m_param_parser_up;
    control_common::utils::ActiveParser m_param_parser_dn;
    control_common::utils::ActiveParser m_param_parser_right;
    control_common::utils::ActiveParser m_param_parser_left;

    /** Track **/
    std::shared_ptr<utils_racing::PathSampled> m_track = std::make_shared<utils_racing::PathSampled>();
    utils_racing::PathAnalyzer m_pathanalyzer;
    double m_closest_track_param{0};
    int m_driving_fwd{1};
    Eigen::Vector2d m_driving_dir{0, 0};
    Eigen::Vector2d m_track_gradient{};
    int m_track_fwd{1};

    /** Path to be tracked **/
    std::shared_ptr<utils_racing::PathSpline> m_path = std::make_shared<utils_racing::PathSpline>();

    utils_racing::LapTimer m_lap_timer;

    /** Control **/
    enum ControlMode
    {
        MANUAL, MPCTUNING, PURSUIT, STANLEY, PIDTRACKING, LAT_ID, LON_ID, RATE
    } m_control_mode{MANUAL};

    enum JoystickMode
    {
        JOYSTICK, TRIGGER, WHEEL
    } p_joystick_mode{JOYSTICK};
    double p_joy_mode_exp{0.0};
    double p_joy_mode_tan{0.0};

    AmzminiCommand command{};
    bool command_valid{false};

    /* MPC tuning */
    struct RacingMetrics
    {
        bool updated{false};
        double performance{0};
        double constraint_margin{999};
    } m_metrics{};

    TimePoint mpc_command_stamp = SteadyClock::now();
    AmzminiCommand mpc_command{};
    bool mpc_armed{false};
    bool mpc_active{false};
    bool mpc_wasActive{false};
    bool evaluate_mpc_active();
    bool mpc_inhibit{false};
    bool mpc_wasInhibit{false};

    void control_drive();

    /* Modular control functions */
    inline double brake_to_stop() { return (vel.norm() > 0.1 ? 1 : 0); }

    /* Turn rate controller */
    double p_turn_rate_FF{0.25};
    double p_turn_rate_P{0.025};
    double p_turn_rate_I{0};
    double turn_rate_err_integral{0};
    void control_rate();

    /* Longitudinal identification */
    void lon_id();
    double p_ident_throttle{0.15};

    /* Lateral identification */
    void lat_id();
    double p_ident_steering_magnitude{radians(5)};
    int m_latphase{0};
    int m_latdir{1};

    /* Path tracking */
    void track_path();
    double p_cross_track_tolerance{0.3};
    PurePursuitControl purePursuitControl;
    double p_lookahead_dist_min{2.0 * amzminiProperties.wheelbase};
    double p_lookahead_time{0.16};
    double p_lookahead_path_param{0.03};
    double p_stanley_gain{1.5};

    /* Longitudinal (speed) control along path */
//    control_common::math::Integral ref_progression{1.0 / p_rate.get_rate()};
//    control_common::math::Integral throttle_integrator{1.0 / p_rate.get_rate()};

    struct ControlDiag
    {
        Vector2 pos_ref{0, 0};
        double rate_ref{0};
        int status{0};
    } m_diag;

    /* Prepare command for publishing */
    double p_master_cap_throttle{0.4};
    void prepareCommand();

private:
    void print_test_res_impl() override
    {
        std::cout << "Publish command: "
                  << command.steering << ", "
                  << command.throttle << ", "
                  << command.brake << "\n";
    }
};

} // namespace amzmini
} // namespace racing
} // namespace control

#endif //SRC_AMZMINICONTROL_HPP
