#ifndef SRC_SPLINEFOLLOWINGTIRELAOPTOCP_HPP
#define SRC_SPLINEFOLLOWINGTIRELAOPTOCP_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/laopt_model/dynamic_bicycle/DynamicBicycle.hpp"
#include "mpc_control_racing/laopt_model/dynamic_bicycle/DynamicBicycleWithActuators.hpp"
#include "mpc_control_racing/laopt_model/dynamic_bicycle/DynamicBicycleWithController.hpp"
#include "mpc_control_racing/laopt_model/AcceleratedPathparam.hpp"
#include "mpc_control_racing/laopt_model/StackedSystem.hpp"

#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#include "mpc_control_racing/problem/spline_following/laopt_ocp/EigenSpline.hpp"
#include "mpc_control_racing/shift_warm_start_heading.hpp"
#include "utils_racing/PathUtils.hpp"

#define USE_U_CMD false
#define USE_LL_CTRL false

#define USE_TRACKBOUNDS true
#define USE_SINGLE_SLACK false

#define USE_TRACKBOUNDS_COST false

#define USE_OBSTACLE_AVOIDANCE false

/* How to invert the direction of following the race path?
 * Insert "1-" before each gamma in the spline_func evaluations
 * path_spline.spline_func(1 - gamma)
 * offsLR_spline.spline_func(1 - gamma)
 */

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {

#if USE_U_CMD
using DynamicBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::DynamicBicycleWithActuators, mpc_control_racing::AcceleratedPathparam>;
#elif USE_LL_CTRL
using DynamicBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::DynamicBicycleWithController, mpc_control_racing::AcceleratedPathparam>;
#else
using DynamicBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::DynamicBicycle, mpc_control_racing::AcceleratedPathparam>;
#endif

class SplineFollowingTireLaoptOcp : public laopt_tools::ControlProblemBase<double /*Scalar*/,
#if USE_U_CMD
        /*NX*/ 8 + 2, /*NU*/ 3>,
#elif (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        /*NX*/ 8, /*NU*/ 3, /*NP*/ 1, /*NG*/ 2, /*NG0*/ 0, /*NGF*/ 0>,
#elif (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        /*NX*/ 8, /*NU*/ 4, /*NP*/ 0, /*NG*/ 2, /*NG0*/ 0, /*NGF*/ 0>,
#else
        /*NX*/ 8, /*NU*/ 3>,
#endif
                                    public mpc_control_racing::RacingOptimizerInterface
{
private:
    DynamicBicycleWithTracking model;

    utils_racing::PathLocator track_locator;
    mpc_control_racing::spline_following::EigenSpline path_spline;
    mpc_control_racing::spline_following::EigenSpline offsLR_spline;

public:
    SplineFollowingTireLaoptOcp()
    {
        /* Define state */
        state_idx.pos_x = 0;
        state_idx.pos_y = 1;
        state_idx.turn_angle = 2;
        state_idx.vel_x = 3;
        state_idx.vel_y = 4;
        state_idx.turn_rate = 5;
        if (USE_U_CMD)
        {
            state_idx.steering = 6;
            state_idx.throttle = 7;
            state_idx.path_param = 8;
            state_idx.path_param_vel = 9;
        }
        else
        {
            state_idx.path_param = 6;
            state_idx.path_param_vel = 7;
        }

        /* Initialize state */
        state.pos = {0, 0};
        state.turn_angle = 0.0;
        state.vel = {0.2, 0.0}; // vx > 0.2
        state.turn_rate = 0.0;
        state.path_param = 0;
        state.path_param_vel = 0;

        if (USE_U_CMD) x_comp.resize(6 + 2 + 2); else x_comp.resize(6 + 2);
        x_comp.setZero();
        x_comp(state_idx.vel_x) = state.vel(0);
        if (USE_U_CMD) x_comp(state_idx.throttle) = state.throttle;

        /* Tuning parameters */
        tuningParameters.w_path_tracking = 50;
        tuningParameters.w_path_progression = 0.1;
        tuningParameters.R_diag = {1e-1, 1e-1, 1e-1};
        tuningParameters.R_du_diag = {1e-1, 1e-3, 1e-3};
        tuningParameters.mayer_factor = 10;

        setup_trajectory_constraints();
    }
    bool init() override
    {
//        State x0; x0 << 0.510, -0.126, -0.207,  0.501, -0.001, -0.001,  0.048,  0.047;
//        Input u; u << 0.4, 0.2, 0.05;
//        Eigen::Vector<double, 0> p;
//        std::cout << "Model evaluation test " << model.dynamics_impl<double>(x0, u, p).transpose() << "\n";
//        return false;
        // Model is identical

//        std::cout << "Spline evaluation test " << spline.eval(0.147).transpose() << " "
//                  << spline.eval(0.151).transpose() << " "
//                  << spline.eval(0.153).transpose() << "\n";
        // Spline is identical

        set_tf(horizon);
        setup_trajectory_constraints();

        update_config_params();
        update_tuning_params();
        configParameters.needs_update = tuningParameters.needs_update = true;
        return true;
    }

    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        // Load common parameters
        RacingOptimizerInterface::load_params_from_yaml(yaml_filepath);

        // Load OCP specific parameters
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        tuningParameters.w_path_tracking = yamlNode["w_path_tracking"].as<double>();
        tuningParameters.w_path_progression = yamlNode["w_path_progression"].as<double>();

        const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        const std::vector<double> R_prev_diag_ = yamlNode["R_prev_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_prev_diag_.size(); i++) { R_prev_diag(i) = R_prev_diag_[i]; }

        const std::vector<double> W_diag_v_body = yamlNode["W_diag_v_body"].as<std::vector<double>>();
        tuningParameters.W_diag_v_body << W_diag_v_body[0], W_diag_v_body[1];

        tuningParameters.w_turn_rate = yamlNode["w_turn_rate"].as<double>();
        tuningParameters.w_slack_contour = yamlNode["w_slack_contour"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Spline Following Tire OCP -- \n"
                  << "w_path_tracking: " << tuningParameters.w_path_tracking << "\n"
                  << "w_path_progression: " << tuningParameters.w_path_progression << "\n"
                  << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "R_prev_diag: " << R_prev_diag.transpose() << "\n"
                  << "W_diag_v_body: " << tuningParameters.W_diag_v_body.transpose() << "\n"
                  << "w_turn_rate: " << tuningParameters.w_turn_rate << "\n"
                  << "w_slack_contour: " << tuningParameters.w_slack_contour << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        path_spline.load_from_yaml(yaml_filepath, "pos");
        offsLR_spline.load_from_yaml(yaml_filepath, "offsLR");
        trackwidth = offsLR_spline.coefs()(0, 3); // Store constant half track width
    }
    /* Updaters for state and parameters */
    Eigen::VectorXd compose_state() override
    {
        state.path_param = track_locator.grid_search(track, state.pos, state.path_param);

        Eigen::Matrix2d T_body_world = get_Tz(state.turn_angle);
        Eigen::Vector2d vel_body = T_body_world * state.vel;
        // std::cout << "vel_body: " << vel_body << std::endl;
        vel_body(0) = std::max(0.501, vel_body(0)); // Project vx onto speed range (lower bound)
        vel_body(1) = std::max(-0.049, std::min(vel_body(1), 0.049)); // Project vy

        // Estimate path param velocity
        state.path_param_vel = vel_body(0) / track->get_info().length;

        // Assemble state from available measurements in state struct
        if (USE_U_CMD)
        {
            if (solve_succeeded)
            {
                // Use previously planned control as initial control for control evolution
                u_prev = get_control();
                state.steering = u_prev(0);
                state.throttle = u_prev(1);
            }
            //            state.throttle = state.vel.norm() / tuningParameters.throttle_to_speed; // throttle to speed is too inaccurate
            const double mid_steering = 0;
            const double mid_throttle = 0.5; // Combined with x0_epsi = 0.499, allows for nominal [0, 1] range
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate, mid_steering, mid_throttle,
                    state.path_param, state.path_param_vel;
        }
        else
        {
            // Assemble state from available measurements in state struct
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate,
                    state.path_param, state.path_param_vel;
        }
        return x_comp;
    }
    void update_config_params() override
    {
#if USE_OBSTACLE_AVOIDANCE
        if (configParameters.obstacle(0) != -99)
        {
            std::cout << " \n\n\n* * * * *\n Update bounds\n";
            const double obstacle_path_param = configParameters.obstacle(0);
            const double obstacle_speed = configParameters.obstacle(2);
            const double obstacle_length = 0.8; // m

            unsigned idx_lower = 0;
            unsigned idx_upper = 0;

            // Find segments to block given obstacle
            for (unsigned i = 0; i < offsLR_spline.breaks().size(); i++)
            {
                if (offsLR_spline.breaks()(i) >= (obstacle_path_param - (obstacle_length / offsLR_spline.get_path_length())))
                {
                    idx_lower = i;
                    break;
                }
            }
            const double lb = offsLR_spline.breaks()(idx_lower);
            const double ub = lb + obstacle_length / offsLR_spline.get_path_length();

            for (unsigned i = idx_lower; i < offsLR_spline.breaks().size(); i++)
            {
                if (offsLR_spline.breaks()(i) >= ub)
                {
                    idx_upper = i;
                    break;
                }
            }
            std::cout << "idx: lw " << idx_lower << ", up: " << idx_upper << "\n";

            // Reset to original values
            offsLR_spline.coefs().rightCols<1>() = Eigen::Vector2d(trackwidth, -trackwidth).replicate(offsLR_spline.coefs().rows() / 2, 1);

//            std::cout << "Reset\n" << offsLR_spline.coefs().rightCols<1>() << "\n";

            // Modify in range
//            constexpr unsigned sideLR{0}; constexpr double signLR{+1};
            constexpr unsigned sideLR{1}; constexpr double signLR{-1};
            for (unsigned i = idx_lower; i < idx_upper; i++)
            {
                offsLR_spline.coefs().row(sideLR + 2 * i).rightCols<1>() = -signLR * 0.2;
            }
//            std::cout << "Set\n" << offsLR_spline.coefs().rightCols<1>() << "\n";
            std::cout << " \n\n\n* * * * *\n\n";
        }
#endif
        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
        u_ub.head<3>() << 1, tuningParameters.throttle_max, 1;
        u_lb.head<3>() << -1, tuningParameters.throttle_min, -1;

        tuningParameters.needs_update = false;
    }

    void setup_trajectory_constraints()
    {
        {
            /* Box constraints on input */
            // steering, throttle, pathparam_acc
            u_ub.head<3>() << 1, tuningParameters.throttle_max, 1;
            u_lb.head<3>() << -1, tuningParameters.throttle_min, -1;
#if (USE_LL_CTRL)
            // Max commanded turnrate
            u_ub(0) = 300.0 * M_PI / 180.0;
            u_lb(0) = -u_ub(0);
#endif
        }

        /* Box constraints on state */
        x_lb(state_idx.vel_x) = 0.2; // vx
//        x_ub(state_idx.vel_x) = 20;  // vx

//        x_lb(state_idx.vel_y) = -5; // vy
//        x_ub(state_idx.vel_y) = 5;  // vy

        x_lb(state_idx.path_param) = 0; // Path parameter can only be positive
        x_ub(state_idx.path_param) = 2;
        x_lb(state_idx.path_param_vel) = 0; // Path parameter can only move forward
        x_ub(state_idx.path_param_vel) = 1;

        xf_lb = x_lb;
        xf_ub = x_ub;

        // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega, path, path vel)
        x0_epsi.resize(x_comp.rows());
        x0_epsi.setZero();

        const Eigen::Vector<double, 6> x0_epsi_dynbcl = {0.005, 0.005, 0.005, 0.01, 0.005, 0.005};
        const Eigen::Vector2d x0_epsi_path = {0.05, 0.01};
        if (USE_U_CMD) { x0_epsi << x0_epsi_dynbcl, 0.999, 0.499, x0_epsi_path; }
        else { x0_epsi << x0_epsi_dynbcl, x0_epsi_path; }

#if USE_TRACKBOUNDS
        {
#if USE_SINGLE_SLACK
            p_lb << 0;
            p_ub << std::numeric_limits<double>::infinity(); //  0.01;
#else
            u_lb(3) = 0;
            u_ub(3) = std::numeric_limits<double>::infinity();
#endif
        }
#endif
    }

    template<typename x_t, typename p_t,
            typename T = typename x_t::Scalar>
    T get_non_control_cost(const Eigen::MatrixBase<x_t> &x,
                           const Eigen::MatrixBase<p_t> &p)
    {
        const Eigen::Vector2<T> pos = {x(state_idx.pos_x), x(state_idx.pos_y)};
        const T gamma = x(state_idx.path_param);
        unused(p);

        T non_control_cost{0};

        const Eigen::Vector2<T> pos_ref = path_spline.eval(gamma);
        const Eigen::Vector2<T> pos_err = pos_ref - pos;
        non_control_cost += tuningParameters.w_path_tracking * pos_err.dot(pos_err);

        const Eigen::Vector2<T> body_vel = {x(state_idx.vel_x), x(state_idx.vel_y)};
        non_control_cost += body_vel.dot(tuningParameters.W_diag_v_body.asDiagonal() * body_vel); //##todo

        const T turn_rate = x(state_idx.turn_rate);
        non_control_cost += tuningParameters.w_turn_rate * turn_rate * turn_rate;

        non_control_cost += -tuningParameters.w_path_progression * (gamma - x_comp(state_idx.path_param));

        /* Using path param velocity cost results in poor numerical solvability! */
        //        const T gamma_vel = x(state_idx.path_param_vel);
        //        non_control_cost += -tuningParameters.w_path_progression * gamma_vel;

        if (USE_TRACKBOUNDS_COST)
        {
            /* Contouring constraints (local linear approximations) */
            //            const T spline_angle = path_spline.eval_tangent(gamma);
            //            const Eigen::Matrix2<T> T_lk = get_Tz(spline_angle);
            //            const Eigen::Vector2<T> pos_diff = pos - pos_ref;
            //            const Eigen::Vector2<T> t_pos_diff = T_lk * pos_diff;
            //            const T t_pos_diff_y = t_pos_diff(1);
            const Eigen::Vector2<T> grad = path_spline.eval_grad(gamma);
            const T t_pos_diff_y = Eigen::RowVector2<T>(-grad(1), grad(0)) * (pos - pos_ref);

            // Constant lateral offset (if spline is centerline)
            const Eigen::Vector2<T> offsLR_ = 0.5 * track->get_info().width * Eigen::Vector2<T>{1, -1};

            // Varying lateral offset (if spline is raceline)
            //            const Eigen::Vector2<T> offsLR_ = offsLR_spline.eval(gamma);

            const double car_width = tuningParameters.car_width;
            const double margin = tuningParameters.lat_acc_max;
            const T wallL = exp(100.0 * (t_pos_diff_y - (offsLR_(0) - 0.5 * car_width - margin)));
            const T wallR = exp(100.0 * (-(-offsLR_(1) - 0.5 * car_width - margin) - t_pos_diff_y));
            non_control_cost += wallL + wallR;
        }
        return non_control_cost;
    }
    template<typename u_t, typename p_t,
            typename T = typename u_t::Scalar>
    T get_control_cost(const Eigen::MatrixBase<u_t> &u,
                       const Eigen::MatrixBase<p_t> &p)
    {
        unused(p);

        T control_cost{0};
        if (USE_U_CMD)
        {
            control_cost = u.dot(tuningParameters.R_du_diag.head<3>().asDiagonal() * u);
        }
        else
        {
            control_cost = u.dot(tuningParameters.R_diag.head<3>().asDiagonal() * u);
        }
        return control_cost;
    }

    template<typename x_t, typename u_t, typename p_t, typename t0_t, typename tf_t,
            typename T = typename x_t::Scalar>
    T lagrange_term_impl(const Eigen::MatrixBase<x_t> &x,
                         const Eigen::MatrixBase<u_t> &u,
                         const Eigen::MatrixBase<p_t> &p,
                         const Eigen::MatrixBase<t0_t> &t0,
                         const Eigen::MatrixBase<tf_t> &tf,
                         const Scalar &tau)
    {
        unused(t0, tf, tau);
        T lagrange_cost = get_non_control_cost(x, p) + get_control_cost(u.template head<3>(), p);

        if (tau < (1.0 / MS_N_SEGS))
        {
            T first_input_cost{0};

            if (USE_U_CMD)
            {
                const Eigen::Vector2<T> input0 = x.template segment<2>(state_idx.steering);
                const Eigen::Vector2<T> diff_prev_input = input0 - u_prev;
                first_input_cost = diff_prev_input.dot(R_prev_diag.asDiagonal() * diff_prev_input);
            }
            else
            {
                const Eigen::Vector2<T> input0 = u.template head<2>();
                const Eigen::Vector2<T> diff_prev_input = input0 - u_prev;
                first_input_cost = diff_prev_input.dot(R_prev_diag.asDiagonal() * diff_prev_input);
            }
            lagrange_cost += first_input_cost;
        }

#if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        const T s_contour = u(3);
        lagrange_cost += tuningParameters.w_slack_contour * (s_contour * s_contour + s_contour);
#endif
        return lagrange_cost;
    }

    template<typename x_t, typename p_t, typename t0_t, typename tf_t,
            typename T = typename x_t::Scalar>
    T mayer_term_impl(const Eigen::MatrixBase<x_t> &xf,
                      const Eigen::MatrixBase<p_t> &p,
                      const Eigen::MatrixBase<t0_t> &t0,
                      const Eigen::MatrixBase<tf_t> &tf)
    {
        unused(t0, tf);

        T mayer_cost{0};
        mayer_cost += tuningParameters.mayer_factor * get_non_control_cost(xf, p);

        /* Using path param progression again in mayer term results in poor yaw stabilization at the beginning of the horizon! */
//        const T gamma_f = xf(state_idx.path_param);
//        const T gamma_vel_f = xf(state_idx.path_param_vel);
//        mayer_cost -=  tuningParameters.w_path_progression * (gamma_f - x_comp(state_idx.path_param));

#if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        const T s_contour = p(0);
        mayer_cost += tuningParameters.w_slack_contour * (s_contour * s_contour + s_contour);
#endif
        return mayer_cost;
    }

    template<typename x_t, typename u_t, typename p_t,
            typename T = typename x_t::Scalar>
    state_t<T> dynamics_impl(const Eigen::MatrixBase<x_t> &x,
                             const Eigen::MatrixBase<u_t> &u,
                             const Eigen::MatrixBase<p_t> &p)
    {
        unused(p);

        DynamicBicycleWithTracking::param_t<T> opt_param; // Empty/independent of p
        return model.dynamics_impl<T>(x, u.template head<3>(), opt_param);
    }

#if USE_TRACKBOUNDS
    template <typename x_t, typename u_t, typename p_t,
              typename T = typename x_t::Scalar> // T is scalar type
    ineq_constr_t<T> inequality_constraints_impl(const Eigen::MatrixBase<x_t> &x,
                                                 const Eigen::MatrixBase<u_t> &u,
                                                 const Eigen::MatrixBase<p_t> &p,
                                                 const Scalar &tau)
    {
        unused(tau);

        ineq_constr_t<T> ineqConstr;
        {
            Eigen::Vector2<T> pos = {x(state_idx.pos_x), x(state_idx.pos_y)};
            const T gamma = x(state_idx.path_param);

            /* Contouring constraints (local linear approximations) */
            const Eigen::Vector2<T> pos_ref = path_spline.eval(gamma);

            /* Contouring constraints (local linear approximations) */
            const T spline_angle = path_spline.eval_tangent(gamma);
//            const Eigen::Matrix2<T> T_lk = get_Tz(spline_angle);
//            const Eigen::Vector2<T> pos_diff = pos - pos_ref;
//            const Eigen::Vector2<T> t_pos_diff = T_lk * pos_diff;
//            const T t_pos_diff_y = t_pos_diff(1);
            const T t_pos_diff_y = Eigen::RowVector2<T>(-sin(spline_angle), cos(spline_angle)) * (pos - pos_ref);
//            const Eigen::Vector2<T> grad = path_spline.eval_grad(gamma);
//            const Eigen::RowVector2<T> cross_grad(-grad(1), grad(0));
//            const T t_pos_diff_y = cross_grad.normalized() * (pos - pos_ref);

#if USE_OBSTACLE_AVOIDANCE
            // Varying lateral offset (if spline is raceline)
            const Eigen::Vector2<T> offsLR_ = offsLR_spline.eval(gamma);
#else
            // Constant lateral offset (if spline is centerline)
            const Eigen::Vector2<T> offsLR_ = 0.5 * track->get_info().width * Eigen::Vector2<T>{1, -1};
#endif

            const double car_width = tuningParameters.car_width;
#if USE_SINGLE_SLACK
            const T s_contour = p(0);
            unused(u);
#else
            const T s_contour = u(3);
            unused(p);
#endif
            ineqConstr(0) = (t_pos_diff_y - (offsLR_(0) - 0.5 * car_width) - s_contour);
            ineqConstr(1) = (-(-offsLR_(1) - 0.5 * car_width) - t_pos_diff_y - s_contour);

//            Eigen::Vector<T, 2> clearanceLR = get_clearanceLR(x, p);
//            ineqConstr(0) = clearanceLR(0) - s_contour;
//            ineqConstr(1) = -clearanceLR(1) - s_contour;
        }
        return ineqConstr;
    }

//    template <typename x_t, typename p_t, typename T = typename x_t::Scalar> // T is scalar type
//    Eigen::Vector<T, 2> get_clearanceLR(const Eigen::MatrixBase<x_t> &x,
//                                        const Eigen::MatrixBase<p_t> &p)
//    {
//        Eigen::Vector2<T> pos = {x(state_idx.pos_x), x(state_idx.pos_y)};
//        const T gamma = x(state_idx.path_param);
//
//        /* Contouring constraints (local linear approximations) */
//        const Eigen::Vector2<T> pos_ref = path_spline.eval(gamma);
//
//        /* Contouring constraints (local linear approximations) */
//        const T spline_angle = path_spline.eval_tangent(gamma);
//        const Eigen::Matrix2<T> T_lk = get_Tz(spline_angle);
//        const Eigen::Vector2<T> pos_diff = pos - pos_ref;
//        const Eigen::Vector2<T> t_pos_diff = T_lk * pos_diff;
//
//        // Constant lateral offset (if spline is centerline)
//        const Eigen::Vector2<T> offsLR_ = 0.5 * track->get_info().width * Eigen::Vector2<T>{1, -1};
//
//        // Varying lateral offset (if spline is raceline)
//        //            const Eigen::Vector2<T> offsLR_ = offsLR_spline.eval(gamma);
//
//        constexpr double car_width = 0.10;
//
//        Eigen::Vector<T, 2> clearanceLR;
//        clearanceLR(0) = t_pos_diff(1) - (offsLR_(0) - 0.5 * car_width);
//        clearanceLR(1) = (-offsLR_(1) - 0.5 * car_width) - t_pos_diff(1);
//    }
#endif

protected:
    Eigen::VectorXd x0_epsi;
    Eigen::Vector2d u_prev{0, 0};
    Eigen::Vector2d R_prev_diag{1e-4, 1e-4};
    bool solve_succeeded{false};

    double trackwidth{0.5};

    template<typename T>
    static Eigen::Matrix2<T> get_Tz(const T &angle)
    {
        Eigen::Matrix2<T> Tz;
        Tz << cos(angle), sin(angle),
                -sin(angle), cos(angle);
        return Tz;
    }

    template<typename Derived>
    bool shift_gamma(Eigen::MatrixBase<Derived> &X)
    {
        // wrap path param
        Eigen::MatrixXd path_param_warm = X.row(state_idx.path_param);
        bool modified = shift_warm_path_param(state.path_param, path_param_warm);
        if (modified) { X.row(state_idx.path_param) = path_param_warm; }
        return modified;
    }
    template<typename Derived>
    bool shift_heading(Eigen::MatrixBase<Derived> &X)
    {
        // wrap turn angle
        Eigen::MatrixXd heading_warm = X.row(state_idx.turn_angle);
        const bool modified = mpc_control_racing::shift_warm_heading(state.turn_angle, heading_warm);
        if (modified) { X.row(state_idx.turn_angle) = heading_warm; }
        return modified;
    }

    Eigen::MatrixXd generate_X_guess(const Eigen::VectorXd &x0, const double dt, const unsigned N_points) const
    {
        Eigen::MatrixXd X_guess = x0.replicate(1, N_points);
        const int px = state_idx.pos_x;
        const int py = state_idx.pos_y;
        const int thet = state_idx.turn_angle;
        const int vx = state_idx.vel_x;
        const int vy = state_idx.vel_y;
        const int w = state_idx.turn_rate;

        for (unsigned i = 0; i < N_points - 1; i++)
        {
            Eigen::VectorXd x = X_guess.col(i);
            Eigen::VectorXd x_next = x;

            // Integrate heading and position
            x_next(thet) += x(w) * dt;
            x_next(px) += (x(vx) * cos(x_next(thet)) - x(vy) * sin(x_next(thet))) * dt; // vx * cos(theta) - vy * sin(theta)
            x_next(py) += (x(vx) * sin(x_next(thet)) + x(vy) * cos(x_next(thet))) * dt; // vx * sin(theta) + vy * cos(theta)

            x_next(state_idx.path_param) += x(state_idx.path_param_vel) * dt; // Integrate path param velocity

            X_guess.col(i + 1) = x_next;
        }
        return X_guess;
    }
};

}
}
}

#endif //SRC_SPLINEFOLLOWINGTIRELAOPTOCP_HPP
