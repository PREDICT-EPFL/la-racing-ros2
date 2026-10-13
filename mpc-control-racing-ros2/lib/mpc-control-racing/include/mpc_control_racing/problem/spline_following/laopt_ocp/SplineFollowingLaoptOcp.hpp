#ifndef SRC_SPLINEFOLLOWINGLAOPTOCP_HPP
#define SRC_SPLINEFOLLOWINGLAOPTOCP_HPP

#define USE_THR_CMD true
#define USE_U_CMD false

#define USE_TRACKBOUNDS true
#define USE_SINGLE_SLACK false

#define USE_VEL_PROFILE_CNSTR true
#define USE_VEL_PROFILE_COST false

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycle.hpp"
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycleWithThrottleActuator.hpp"
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycleWithActuators.hpp"
#include "mpc_control_racing/laopt_model/AcceleratedPathparam.hpp"
#include "mpc_control_racing/laopt_model/StackedSystem.hpp"

#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#include "mpc_control_racing/problem/spline_following/laopt_ocp/EigenSpline.hpp"
#if (USE_VEL_PROFILE_CNSTR || USE_VEL_PROFILE_COST)
#include "mpc_control_racing/problem/spline_following/laopt_ocp/EigenVelocityProfileSplineLA.hpp"
#endif
#include "mpc_control_racing/shift_warm_start_heading.hpp"
#include "utils_racing/PathUtils.hpp"

#include "mpc_control_common/ControlMath.hpp"

namespace mpc_control_racing {
namespace spline_following {
namespace laopt_ocp {

#if USE_U_CMD
using SimpleBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::SimpleBicycleWithActuators,
                                                                    mpc_control_racing::AcceleratedPathparam>;
class SplineFollowingLaoptOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 5+2,
#elif USE_THR_CMD
using SimpleBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::SimpleBicycleWithThrottleActuator,
                                                                    mpc_control_racing::AcceleratedPathparam>;
class SplineFollowingLaoptOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 4+2,
#else
using SimpleBicycleWithTracking = mpc_control_racing::StackedSystem<mpc_control_racing::SimpleBicycle,
                                                                    mpc_control_racing::AcceleratedPathparam>;
class SplineFollowingLaoptOcp : public laopt_tools::ControlProblemBase</*Scalar*/ double, /*NX*/ 3+2,
#endif

#if ((USE_TRACKBOUNDS && USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
// Trackbounds: 1 global param, 2 constraints | Velocity constraint: 1 global param, 1 constraint (also at t0)
                                                                       /*NU*/ 3, /*NP*/ 1+1, /*NG*/ 2+1, /*NG0*/ 1, /*NGF*/ (USE_U_CMD || USE_THR_CMD) ? 1 : 0>,
#elif ((USE_TRACKBOUNDS && !USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
// Trackbounds: 1 extra input, 2 constraints | Velocity constraint: 1 global param, 1 constraint (also at t0)
                                                                       /*NU*/ 3+1, /*NP*/ 1, /*NG*/ 2+1, /*NG0*/ 1, /*NGF*/ (USE_U_CMD || USE_THR_CMD) ? 1 : 0>,
#elif (USE_TRACKBOUNDS && USE_SINGLE_SLACK) // Trackbounds: 1 global param, 2 constraints
                                                                       /*NU*/ 3, /*NP*/ 1, /*NG*/ 2, /*NG0*/ 0, /*NGF*/ 0>,
#elif (USE_TRACKBOUNDS && !USE_SINGLE_SLACK) // Trackbounds: 1 extra input, 2 constraints
                                                                       /*NU*/ 3+1, /*NP*/ 0, /*NG*/ 2, /*NG0*/ 0, /*NGF*/ 0>,
#elif (USE_VEL_PROFILE_CNSTR)
// Velocity constraint: 1 global param, 1 constraint (also at t0)
                                                                       /*NU*/ 3, /*NP*/ 1, /*NG*/ 1, /*NG0*/ 1, /*NGF*/ (USE_U_CMD || USE_THR_CMD) ? 1 : 0>,
#else // Base (no optimization params, no constraints)
                                                                       /*NU*/ 3, /*NP*/ 0, /*NG*/ 0, /*NG0*/ 0, /*NGF*/ 0>,
#endif
                                public mpc_control_racing::RacingOptimizerInterface
{
protected:
    utils_racing::PathLocator track_locator;
    mpc_control_racing::spline_following::EigenSpline path_spline;
    mpc_control_racing::spline_following::EigenSpline offsLR_spline;
#if USE_VEL_PROFILE_CNSTR || USE_VEL_PROFILE_COST
    mpc_control_racing::spline_following::EigenVelocityProfileLA velocity_profile{true};
#endif
    SimpleBicycleWithTracking model;

public:
    SplineFollowingLaoptOcp()
    {
        /* Define state */
        state_idx.pos_x = 0;
        state_idx.pos_y = 1;
        state_idx.turn_angle = 2;
        if (USE_U_CMD)
        {
            state_idx.steering = 3;
            state_idx.throttle = 4;
            state_idx.path_param = 5;
            state_idx.path_param_vel = 6;
        }
        else if (USE_THR_CMD)
        {
            state_idx.throttle = 3;
            state_idx.path_param = 4;
            state_idx.path_param_vel = 5;
        }
        else
        {
            state_idx.path_param = 3;
            state_idx.path_param_vel = 4;
        }

        /* Initialize state */
        state.pos = {0, 0};
        state.turn_angle = 0;
        state.path_param = 0;
        state.path_param_vel = 0;
        if (USE_U_CMD)
        {
            state.steering = 0;
            state.throttle = 0;
        }
        else if (USE_THR_CMD)
        {
            state.throttle = 0;
        }

        if (USE_U_CMD) { x_comp.resize(5 + 2); }
        else if (USE_THR_CMD) { x_comp.resize(4 + 2); }
        else { x_comp.resize(3 + 2); }

        x_comp.setZero();
        if (USE_U_CMD || USE_THR_CMD) { x_comp(state_idx.throttle) = state.throttle; }

        /* Tuning parameters */
        tuningParameters.w_path_tracking = 50;
        tuningParameters.w_path_progression = 0.1;
        tuningParameters.R_diag = {1e-1, 1e-3, 1e-3};
        tuningParameters.mayer_factor = 10;
        tuningParameters.throttle_to_speed = 8;
    }
    bool init()
    {
//        State x0; x0 << 0.510, -0.126, -0.207, 0.3, 0.15;
//        Input u; u << 0.4, 0.2, 0.05;
//        Param p;
//        std::cout << "Model evaluation test " << dynamics_impl(x0, u, p).transpose() << "\n";
        /// Model is identical

        set_tf(horizon);
        setup_trajectory_constraints();

        update_config_params();
        update_tuning_params();
        configParameters.needs_update = tuningParameters.needs_update = true;
        return true;
    }

    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        /* Load common parameters */
        RacingOptimizerInterface::load_params_from_yaml(yaml_filepath);

        /* Load OCP specific parameters */
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        tuningParameters.w_path_tracking = yamlNode["w_path_tracking"].as<double>();
        tuningParameters.w_path_progression = yamlNode["w_path_progression"].as<double>();

        tuningParameters.w_slack_contour = yamlNode["w_slack_contour"].as<double>();

        // const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        // for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        const std::vector<double> R_prev_diag_ = yamlNode["R_prev_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_prev_diag_.size(); i++) { R_prev_diag(i) = R_prev_diag_[i]; }

        tuningParameters.throttle_to_speed = yamlNode["throttle_to_speed"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Spline Following OCP -- \n"
                  << "w_path_tracking: " << tuningParameters.w_path_tracking << "\n"
                  << "w_path_progression: " << tuningParameters.w_path_progression << "\n"
                  << "w_slack_contour: " << tuningParameters.w_slack_contour << "\n"
                  // << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "R_prev_diag: " << R_prev_diag.transpose() << "\n"
                  << "throttle_to_speed: " << tuningParameters.throttle_to_speed << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        path_spline.load_from_yaml(yaml_filepath, "pos");
        offsLR_spline.load_from_yaml(yaml_filepath, "offsLR");
        trackwidth = offsLR_spline.coefs()(0, 3); // Store constant half track width
    }
    void load_velocity_profile_from_yaml(const std::string &yaml_filepath)
    {
#if USE_VEL_PROFILE_CNSTR || USE_VEL_PROFILE_COST
        velocity_profile.load_from_yaml(yaml_filepath);
#else
        (void) yaml_filepath;
#endif
    }

    /* Updaters for state and parameters */
    Eigen::VectorXd compose_state() override
    {
        state.path_param = track_locator.grid_search(track, state.pos, state.path_param);

        Eigen::Matrix2d T_body_world = get_Tz(state.turn_angle);
        Eigen::Vector2d vel_body = T_body_world * state.vel;

        vel_body(0) = std::max(0.501, vel_body(0)); // Project vx onto speed range (lower bound)
        vel_body(1) = std::max(-0.049, std::min(vel_body(1), 0.049)); // Project vy

        state.path_param_vel = vel_body(0) / path_spline.get_path_length();

        /* Assemble state from available measurements in state struct */
        if (USE_THR_CMD)
        {
            double throttle_guess{0};
            const double throttle_guess_meas = state.vel.norm() / tuningParameters.throttle_to_speed; // throttle to speed is too inaccurate
//            std::cout << "throttle_guess (meas): " << throttle_guess_meas << "\n";

            throttle_guess = throttle_guess_meas;

            /* As soon as available, use previous (predicted) inputs as actuator state */
            if (solve_succeeded)
            {
//                u_prev = get_control().topRows<2>(); // Previously commanded inputs
//                const double throttle_guess_plnd = get_state_trajectory()(state_idx.throttle, 1); // Predicted throttle after 1 step
//                std::cout << "throttle_guess (plnd): " << throttle_guess_plnd << "\n";

                /* Don't use this because solve is not always succeeded, so it jumps up and down */
//                throttle_guess += control_math::constrain(throttle_guess_plnd - throttle_guess_meas, 0.2);
            }
            state.throttle = throttle_guess; // For consistency, not used further

            x_comp << state.pos, state.turn_angle, throttle_guess,
                    state.path_param, state.path_param_vel;
        }
        else if (USE_U_CMD)
        {

            double steering_guess = 0;
            double throttle_guess = state.vel.norm() / tuningParameters.throttle_to_speed; // throttle to speed is too inaccurate

            /* As soon as available, use previous (predicted) inputs as actuator state */
            if (solve_succeeded)
            {
                u_prev = get_control().topRows<2>(); // Previously commanded inputs
                steering_guess = get_state_trajectory()(state_idx.steering, 1); // Predicted steering
                throttle_guess = get_state_trajectory()(state_idx.throttle, 1); // Predicted throttle after 1 step
                std::cout << "steering_thr_guess (prd): " << steering_guess << ", " << throttle_guess << "\n";
            }
            else { std::cout << "throttle_guess (vel): " << throttle_guess << "\n"; }
            state.steering = steering_guess; // For consistency
            state.throttle = throttle_guess;

            x_comp << state.pos, state.turn_angle,
                    steering_guess, throttle_guess,
                    state.path_param, state.path_param_vel;
        }
        else
        {
            x_comp << state.pos, state.turn_angle, state.path_param,
                    state.path_param_vel;
        }
        return x_comp;
    }
    void update_config_params() override
    {
        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
#if USE_VEL_PROFILE_CNSTR || USE_VEL_PROFILE_COST
        if (configParameters.path_params.size() > 0)
        {
            velocity_profile.collocation_points = configParameters.path_params;
            velocity_profile.max_velocity = configParameters.path_velocities;

            velocity_profile.D_poly = 1;
            velocity_profile.N_seg = std::round(configParameters.path_params.size() - 1) / velocity_profile.D_poly;
            std::cout << "Vel profile: N_segs: " << velocity_profile.N_seg << ", D_poly: " << velocity_profile.D_poly << "\n";
            velocity_profile.init();
        }
#endif
        tuningParameters.needs_update = false;
    }

    void setup_trajectory_constraints()
    {
        {
            /* Box constraints on input */
            // steering, throttle, pathparam_acc
            u_ub.head<3>() << 1, tuningParameters.throttle_max, 1;
            u_lb.head<3>() << -1, tuningParameters.throttle_min, -1;
        }

        /* Box constraints on state */
        x_lb(state_idx.path_param) = 0; // Path parameter can only be positive
        x_ub(state_idx.path_param) = 2;
        x_lb(state_idx.path_param_vel) = 0; // Path parameter can only move forward
        x_ub(state_idx.path_param_vel) = 1;

//        if (USE_U_CMD || USE_THR_CMD)
//        {
//            // Only need this for x0 throttle !?
//            x_lb(state_idx.throttle) = tuningParameters.throttle_min;
//            x_ub(state_idx.throttle) = tuningParameters.throttle_max;
//        }

        xf_lb = x_lb;
        xf_ub = x_ub;

        // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega, path, path vel)
        x0_epsi.resize(x_comp.rows());
        x0_epsi.setZero();

        const Eigen::Vector3d x0_epsi_kinbcl = {0.005, 0.005, 0.005};
        const Eigen::Vector2d x0_epsi_path = {0.05, 0.1};
        if (USE_THR_CMD) { x0_epsi << x0_epsi_kinbcl, (1 - 0), x0_epsi_path; } // Initial throttle state is free
        else if (USE_U_CMD) { x0_epsi << x0_epsi_kinbcl, (1 -(-1))/100, (1 - 0)/100, x0_epsi_path; }
        else { x0_epsi << x0_epsi_kinbcl, x0_epsi_path; }

        // Boundary conditions: Optimization parameters
        if ((USE_TRACKBOUNDS && USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
        {
            // Slack for trackbounds is param
            p_lb(0) = 0;
            p_ub(0) = std::numeric_limits<double>::infinity();
            // Slack for velocity constraint
            p_lb(1) = 0;
            p_ub(1) = std::numeric_limits<double>::infinity();
        }
        else if ((USE_TRACKBOUNDS && !USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
        {
            // Slack for trackbounds is input
            u_lb(3) = 0;
            u_ub(3) = std::numeric_limits<double>::infinity();
            // Slack for velocity constraint
            p_lb(0) = 0;
            p_ub(0) = std::numeric_limits<double>::infinity();
        }
        else if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        {
            // Slack for trackbounds is param
            p_lb(0) = 0;
            p_ub(0) = std::numeric_limits<double>::infinity();
        }
        else if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        {
            // Slack for trackbounds is input
            u_lb(3) = 0;
            u_ub(3) = std::numeric_limits<double>::infinity();
        }
        else if (USE_VEL_PROFILE_CNSTR)
        {
            // Slack for velocity constraint
            p_lb(0) = 0;
            p_ub(0) = std::numeric_limits<double>::infinity();
        }
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

        non_control_cost += -tuningParameters.w_path_progression * (gamma - state.path_param);

        return non_control_cost;
    }
    template<typename u_t, typename p_t,
            typename T = typename u_t::Scalar>
    T get_control_cost(const Eigen::MatrixBase<u_t> &u,
                       const Eigen::MatrixBase<p_t> &p)
    {
        unused(p);

        T control_cost{0};
        /* If U_CMD or THR_CMD, it is the same cost on them as the unit doesn't change */
        control_cost += u.dot(tuningParameters.R_diag.head<3>().asDiagonal() * u);

        // // Only add cost on >>OCP<< inputs. Inputs that are states in the OCP are handled in lagrange_term
        // if (USE_U_DOT)
        // {
        //     const Eigen::Vector2<T> u_car = u.template topRows<2>();
        //     control_cost += u_car.dot(tuningParameters.R_du_diag.head<2>().asDiagonal() * u_car);
        //     control_cost += tuningParameters.R_diag(2) * u(2) * u(2); // Path accel
        // }
        // else if (USE_ACC)
        // {
        //     control_cost += tuningParameters.R_diag(0) * u(0) * u(0);
        //     control_cost += tuningParameters.R_du_diag(1) * u(1) * u(1);
        //     control_cost += tuningParameters.R_diag(2) * u(2) * u(2); // Path accel
        // }
        // else
        // {
        // control_cost += u.dot(tuningParameters.R_diag.head<3>().asDiagonal() * u);
        // }
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
        unused(p, t0, tf, tau);
        T lagrange_cost = get_non_control_cost(x, p) + get_control_cost(u.template head<3>(), p);

#if USE_VEL_PROFILE_COST
        {
            const T gamma = x(state_idx.path_param);
            const T throttle_max = velocity_profile.eval(gamma) / tuningParameters.throttle_to_speed; // for now. later switch to throttle

            T profile_cost{0};
            if (USE_THR_CMD || USE_U_CMD)
            {
                profile_cost += -0.1 * log(-(x(state_idx.throttle) - throttle_max));
            }
            else
            {
                profile_cost += -0.1 * log(-(u(1) - throttle_max));
            }
            lagrange_cost += profile_cost;
        }
#endif

        // // Add cost on inputs that are states in the OCP
        // if (USE_U_CMD)
        // {
        //     const Eigen::Vector2<T> u_phys = {x(state_idx.steering), x(state_idx.throttle)};
        //     lagrange_cost += u_phys.dot(tuningParameters.R_diag.head<2>().asDiagonal() * u_phys);
        // }
        // else if (USE_THR_CMD)
        // {
        //     const T throttle = x(state_idx.throttle);
        //     lagrange_cost += tuningParameters.R_diag(0) * throttle * throttle;
        // }

        if (tau < (1.0 / MS_N_SEGS)) // std::cout << "tau = " << tau << "\n";
        {
            T first_input_cost{0};

            /* On inputs */
            const Eigen::Vector2<T> input0 = u.template head<2>();
            const Eigen::Vector2<T> diff_prev_input = input0 - u_prev;
            first_input_cost += diff_prev_input.dot(R_prev_diag.asDiagonal() * diff_prev_input);

            /* On inputs in states (in addition) */
            if (USE_U_CMD)
            {
//                const Eigen::Vector2<T> input0_state = x.template segment<2>(state_idx.steering);
                const Eigen::Vector2<T> diff_input0_state = input0 - u_prev;
                first_input_cost += diff_input0_state.dot(R_prev_diag.asDiagonal() * diff_input0_state);
            }
            else if (USE_THR_CMD)
            {
                const T throttle0_state = x(state_idx.throttle);
                const T diff_throttle0_state = throttle0_state - u_prev(1);
                first_input_cost += R_prev_diag(1) * diff_throttle0_state * diff_throttle0_state;
            }
            lagrange_cost += first_input_cost;
        }

        if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        {
            const T s_contour = u(3);
            lagrange_cost += tuningParameters.w_slack_contour * (s_contour * s_contour + s_contour);
        }
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
        mayer_cost = tuningParameters.mayer_factor * get_non_control_cost(xf, p);

        /* Using path param progression again in mayer term results in poor yaw stabilization at the beginning of the horizon! */
//        const T gamma_f = xf(state_idx.path_param);
//        const T gamma_vel_f = xf(state_idx.path_param_vel);
//        mayer_cost -=  tuningParameters.w_path_progression * (gamma_f - x_comp(state_idx.path_param));

        T s_contour;
        T s_throttle;
        if ((USE_TRACKBOUNDS && USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
        {
            // Slack for trackbounds is param
            s_contour = p(0);
            // Slack for velocity constraint
            s_throttle = p(1);
        }
        else if ((USE_TRACKBOUNDS && !USE_SINGLE_SLACK) && USE_VEL_PROFILE_CNSTR)
        {
            // Slack for trackbounds is input -- handled in lagrange cost!
            // Slack for velocity constraint
            s_throttle = p(0);
        }
        else if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        {
            // Slack for trackbounds is param
            s_contour = p(0);
        }
        else if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        {
            // Slack for trackbounds is input -- handled in lagrange cost!
        }
        else if (USE_VEL_PROFILE_CNSTR)
        {
            // Slack for velocity constraint
            s_throttle = p(0);
        }

        if (USE_TRACKBOUNDS && USE_SINGLE_SLACK) { mayer_cost += tuningParameters.w_slack_contour * (s_contour * s_contour + s_contour); }
        if (USE_VEL_PROFILE_CNSTR) { mayer_cost += 1e4 * (s_throttle * s_throttle + s_throttle); }

        return mayer_cost;
    }

    template<typename x_t, typename u_t, typename p_t,
            typename T = typename x_t::Scalar>
    state_t<T> dynamics_impl(const Eigen::MatrixBase<x_t> &x,
                             const Eigen::MatrixBase<u_t> &u,
                             const Eigen::MatrixBase<p_t> &p)
    {
        (void) p;
        SimpleBicycleWithTracking::param_t<T> opt_param; // Empty/independent of p
        return model.dynamics_impl<T>(x, u.template head<3>(), opt_param);
    }

    template <typename x_t, typename u_t, typename p_t,
              typename T = typename x_t::Scalar> // T is scalar type
    ineq_constr_t<T> inequality_constraints_impl(const Eigen::MatrixBase<x_t>& x,
                                                 const Eigen::MatrixBase<u_t>& u,
                                                 const Eigen::MatrixBase<p_t>& p,
                                                 const Scalar& tau)
    {
        const T gamma = x(state_idx.path_param);

        ineq_constr_t<T> ineqConstr;

        if (USE_TRACKBOUNDS)
        {
            T s_contour;
            if (USE_SINGLE_SLACK) { s_contour = p(0); }
            else { s_contour = u(3); }

            Eigen::Vector2<T> pos = {x(state_idx.pos_x), x(state_idx.pos_y)};
            Eigen::Vector<T, 2> contouringConstr = get_contouring_constraints(pos, gamma, s_contour);
            ineqConstr(0) = contouringConstr(0);
            ineqConstr(1) = contouringConstr(1);
        }

#if (USE_VEL_PROFILE_CNSTR) // Need compile-time due velocity_profile member
        {
            const T throttle_max = velocity_profile.eval(gamma) / tuningParameters.throttle_to_speed; // for now. later switch to throttle
            T s_throttle;
            if (USE_TRACKBOUNDS && USE_SINGLE_SLACK) { s_throttle = p(1); }
            else { s_throttle = p(0); }

            if (USE_THR_CMD || USE_U_CMD)
            {
                ineqConstr(NG-1) = (x(state_idx.throttle) - throttle_max - s_throttle);
            }
            else
            {
                ineqConstr(NG-1) = (u(1) - throttle_max - s_throttle);
            }
        }
#endif
        return ineqConstr;
    }

    template <typename x_t, typename u_t, typename p_t,
              typename T = typename x_t::Scalar> // T is scalar type
    ineq_constr0_t<T> inequality_constraints0_impl(const Eigen::MatrixBase<x_t> &x0,
                                                   const Eigen::MatrixBase<u_t> &u0,
                                                   const Eigen::MatrixBase<p_t>& p)
    {
        ineq_constr0_t<T> ineqConstr0;

#if (USE_VEL_PROFILE_CNSTR) // Need compile-time due velocity_profile member
        {
            const T gamma = x0(state_idx.path_param);
            const T throttle_max = velocity_profile.eval(gamma) / tuningParameters.throttle_to_speed; // for now. later switch to throttle

            T s_throttle;
            if (USE_TRACKBOUNDS && USE_SINGLE_SLACK) { s_throttle = p(1); }
            else { s_throttle = p(0); }
            if (USE_THR_CMD || USE_U_CMD)
            {
                ineqConstr0(NG0-1) = (x0(state_idx.throttle) - throttle_max - s_throttle);
            }
            else
            {
                ineqConstr0(NG0-1) = (u0(1) - throttle_max - s_throttle);
            }
        }
#endif
        return ineqConstr0;
    }

    template <typename x_t, typename p_t,
              typename T = typename x_t::Scalar> // T is scalar type
    ineq_constrf_t<T> inequality_constraintsf_impl(const Eigen::MatrixBase<x_t>& xf,
                                                  const Eigen::MatrixBase<p_t>& p)
    {
        ineq_constrf_t<T> ineqConstrf;

#if (USE_VEL_PROFILE_CNSTR) // Need compile-time due velocity_profile member
        {
            const T gamma = xf(state_idx.path_param);
            const T throttle_max = velocity_profile.eval(gamma) / tuningParameters.throttle_to_speed; // for now. later switch to throttle

            T s_throttle;
            if (USE_TRACKBOUNDS && USE_SINGLE_SLACK) { s_throttle = p(1); }
            else { s_throttle = p(0); }
            if (USE_THR_CMD || USE_U_CMD)
            {
                ineqConstrf(NG0-1) = (xf(state_idx.throttle) - throttle_max - s_throttle);
            }
        }
#endif
        return ineqConstrf;
    }

    template<typename T>
    Eigen::Vector<T, 2> get_contouring_constraints(const Eigen::Vector<T, 2> &pos,
                                                   const T &gamma, const T &s_contour)
    {
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

        // Constant lateral offset (if spline is centerline)
        const Eigen::Vector2<T> offsLR_ = 0.5 * track->get_info().width * Eigen::Vector2<T>{1, -1};

        // Varying lateral offset (if spline is raceline)
        //            const Eigen::Vector2<T> offsLR_ = offsLR_spline.eval(gamma);

        const double car_width = tuningParameters.car_width;

        Eigen::Vector<T, 2> contouringConstr;
        contouringConstr(0) = (t_pos_diff_y - (offsLR_(0) - 0.5 * car_width) - s_contour);
        contouringConstr(1) = (-(-offsLR_(1) - 0.5 * car_width) - t_pos_diff_y - s_contour);

        return contouringConstr;
    }

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
        bool modified =  mpc_control_racing::spline_following::shift_warm_path_param(state.path_param, path_param_warm);
        if (modified) { X.row(state_idx.path_param) = path_param_warm; }
        return modified;
    }
    template<typename Derived>
    bool shift_heading(Eigen::MatrixBase<Derived> &X)
    {
        // wrap turn angle
        Eigen::MatrixXd heading_warm = X.row(state_idx.turn_angle);
        bool modified = mpc_control_racing::shift_warm_heading(state.turn_angle, heading_warm);
        if (modified) { X.row(state_idx.turn_angle) = heading_warm; }
        return modified;
    }

    Eigen::MatrixXd generate_X_guess(const Eigen::VectorXd &x0, double dt, unsigned N_points)
    {
        Eigen::MatrixXd X_guess = x0.replicate(1, N_points);
        int px = state_idx.pos_x;
        int py = state_idx.pos_y;
        int thet = state_idx.turn_angle;

        double V = 2;

        for (unsigned i = 0; i < N_points - 1; i++)
        {
            Eigen::VectorXd x = X_guess.col(i);
            Eigen::VectorXd x_next = x;

            // Integrate position
            x_next(px) += (V * cos(thet)) * dt;
            x_next(py) += (V * sin(thet)) * dt;

            X_guess.col(i + 1) = x_next;
        }
        return X_guess;
    }

};

}
}
}

#endif //SRC_SPLINEFOLLOWINGLAOPTOCP_HPP
