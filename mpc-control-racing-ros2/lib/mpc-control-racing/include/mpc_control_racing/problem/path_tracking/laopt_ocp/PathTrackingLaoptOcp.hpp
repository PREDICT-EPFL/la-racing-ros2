#ifndef SRC_PATHTRACKINGLAOPTOCP_HPP
#define SRC_PATHTRACKINGLAOPTOCP_HPP

#include "laopt/laopt.hpp"
#include "laopt/tools/control_problem_base.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycle.hpp"
#include "mpc_control_racing/laopt_model/kinematic_bicycle/SimpleBicycleWithActuators.hpp"
#include "mpc_control_racing/laopt_model/kinematic_bicycle/AcceleratedBicycle.hpp"

#include "mpc_control_racing/problem/path_tracking/PathTrackingTranscriptionSettings.hpp"

#include "mpc_control_racing/problem/spline_following/laopt_ocp/EigenSpline.hpp"
#include "mpc_control_racing/shift_warm_start_heading.hpp"
#include "utils_racing/PathUtils.hpp"

#define USE_U_CMD false
#define USE_ACCEL false
#define USE_TRACKBOUNDS false

namespace mpc_control_racing {
namespace path_tracking {
namespace laopt_ocp {

#if USE_U_CMD
using Model = mpc_control_racing::SimpleBicycleWithActuators;
class PathTrackingLaoptOcp : public laopt_tools::ControlProblemBase<double /*Scalar*/, 3+2 /*NX*/, 2 /*NU*/>,
#elif USE_ACCEL
using Model = mpc_control_racing::AcceleratedBicycle;
class PathTrackingLaoptOcp : public laopt_tools::ControlProblemBase<double /*Scalar*/, 4 /*NX*/, 2 /*NU*/>,
#elif USE_TRACKBOUNDS
using Model = mpc_control_racing::SimpleBicycle;
class PathTrackingLaoptOcp
        : public laopt_tools::ControlProblemBase<double /*Scalar*/, 3 /*NX*/, 2 /*NU*/, 1 /*NP*/, 2 /*NG*/>,
#else
using Model = mpc_control_racing::SimpleBicycle;
class PathTrackingLaoptOcp
        : public laopt_tools::ControlProblemBase<double /*Scalar*/, 3 /*NX*/, 2 /*NU*/, 0 /*NP*/, 0 /*NG*/>,
#endif
          public mpc_control_racing::RacingOptimizerInterface
{
private:
    Model model;

    utils_racing::PathLocator track_locator;
    mpc_control_racing::spline_following::EigenSpline spline;

public:
    PathTrackingLaoptOcp()
    {
//        State x0;
//        if (USE_U_CMD) x0 << 0.510, -0.126, -0.207, 0.501, -0.001, -0.001, 0.3, 0.15;
//        else x0 << 0.510, -0.126, -0.207, 0.501, -0.001, -0.001;
//        Input u; u << 0.4, 0.2;
//        Param p;
//        std::cout << "Model evaluation test " << model.dynamics_impl<double>(x0, u, p).transpose() << "\n";
        // Model is identical

        Eigen::Vector2d tangent;
        tangent << 0.3, 0.1;
        std::cout << "tangent: " << tangent.transpose() << "\nmat: " << gradToTransMat(tangent) << "\n";

        set_tf(horizon);

        /* Define state */
        state_idx.pos_x = 0;
        state_idx.pos_y = 1;
        state_idx.turn_angle = 2;
        if (USE_ACCEL) { state_idx.vel_x = 3; }
        else if (USE_U_CMD)
        {
            state_idx.steering = 3;
            state_idx.throttle = 4;
        }

        /* Initialize state */
        state.pos = {0, 0};
        state.turn_angle = 0;
        if (USE_ACCEL) { state.vel(0) = 0.5; }
        else if (USE_U_CMD)
        {
            state.steering = 0;
            state.throttle = 0.2;
        }

        if (USE_ACCEL) { x_comp.resize(3 + 1); }
        else if (USE_U_CMD) { x_comp.resize(3 + 2); }
        else { x_comp.resize(3); }

        x_comp.setZero();
        if (USE_U_CMD) { x_comp(state_idx.throttle) = state.throttle; }

        /* Config parameters */
        configParameters.path_params.resize(MS_N_SEGS + 1);
        configParameters.path_velocities.resize(MS_N_SEGS + 1);
        configParameters.raceline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.grad_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_grad_refs.resize(MS_N_SEGS + 1, 2);

        configParameters.path_params = Eigen::VectorXd::LinSpaced(MS_N_SEGS + 1, 0, 0.1);
        configParameters.raceline_refs.setZero();
        configParameters.grad_refs.setZero();
        configParameters.centerline_refs.setZero();
        configParameters.centerline_grad_refs.setZero();

        /* Tuning parameters default */
        tuningParameters.W_diag_path_tracking << 1, 1;
        tuningParameters.w_speed_tracking = 1;
        tuningParameters.R_diag.topRows(2) << 1, 1;
        tuningParameters.mayer_factor = 1;

        setup_trajectory_constraints();
    }
    bool init() override
    {
//        State x0; x0 << 0.510, -0.126, -0.207;
//        Input u; u << 0.4, 0.2;
//        Param p;
//        std::cout << "Model evaluation test " << model.dynamics_impl<double>(x0, u, p).transpose() << "\n";
        /// Model is identical

//        std::cout << "Spline evaluation test " << spline.eval(0.147).transpose() << " "
//                  << spline.eval(0.151).transpose() << " "
//                  << spline.eval(0.153).transpose() << "\n";
        // Spline is identical

        set_tf(horizon);
        setup_trajectory_constraints();

        for (unsigned i = 0; i < (MS_N_SEGS + 1); i++)
        {
            const Eigen::Vector2d pos = spline.eval(configParameters.path_params(i));
            const Eigen::Vector2d grad = spline.eval_grad(configParameters.path_params(i));
            configParameters.raceline_refs.row(i) = pos.transpose();
            configParameters.grad_refs.row(i) = grad.normalized().transpose();
        }
        return true;
    }

    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        // Load common parameters
        RacingOptimizerInterface::load_params_from_yaml(yaml_filepath);

        // Load OCP specific parameters
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        const std::vector<double> W_diag = yamlNode["W_diag_path_tracking"].as<std::vector<double>>();
        tuningParameters.W_diag_path_tracking << W_diag[0], W_diag[1];
        tuningParameters.w_speed_tracking = yamlNode["w_speed_tracking"].as<double>();

        const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        tuningParameters.lon_acc_max = yamlNode["lon_acc_max"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Path Tracking OCP -- \n"
                  << "W_diag_path_tracking: " << tuningParameters.W_diag_path_tracking.transpose() << "\n"
                  << "w_speed_tracking: " << tuningParameters.w_speed_tracking << "\n"
                  << "lon_acc_max: " << tuningParameters.lon_acc_max << "\n"
                  << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "\n";
    }

    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        spline.load_from_yaml(yaml_filepath);
        init();
    }

    /* Updaters for state and parameters */
    Eigen::VectorXd compose_state() override
    {
        // Assemble state from available measurements in state struct
        if (USE_ACCEL) { x_comp << state.pos, state.turn_angle, state.vel(0); }
        else if (USE_U_CMD) { x_comp << state.pos, state.turn_angle, state.turn_angle, state.throttle; }
        else { x_comp << state.pos, state.turn_angle; }
        return x_comp;
    }
    void update_config_params() override
    {
        // Gradient normalization is performed in cost function
//        for (unsigned i = 0; i < MS_N_SEGS + 1; i++) { configParameters.grad_refs.row(i).normalize(); }

        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
        set_tf(horizon);

        u_ub << 1, tuningParameters.throttle_max;
        u_lb << -1, tuningParameters.throttle_min;

        RacingOcpParams::print_params();
        tuningParameters.needs_update = false;
    }

    void setup_trajectory_constraints()
    {
        /* Box constraints on input */
        {
            // steering, throttle, pathparam_acc
            u_ub << 1, tuningParameters.throttle_max;
            u_lb << -1, tuningParameters.throttle_min;
        }

        /* Box constraints on state */
        {

        }
        xf_lb = x_lb;
        xf_ub = x_ub;

        // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega, path, path vel)
        x0_epsi.resize(x_comp.rows());
        x0_epsi.setZero();

        if (USE_ACCEL) { x0_epsi << 0.001, 0.001, 0.002, 0.001; }
        else if (USE_U_CMD) { x0_epsi << 0.001, 0.001, 0.002, 0.1, 0.1; }
        else { x0_epsi << 0.001, 0.001, 0.002; }

        // Slack variable
        if (USE_TRACKBOUNDS)
        {
            p_lb << 0;
            p_ub << 0.5;
        }
    }

    static Eigen::Matrix2d gradToTransMat(const Eigen::Vector2d &grad)
    {
        // Return transformation matrix from track frame to local tangent frame
        Eigen::Vector2d orth;
        orth << -grad(1), grad(0);

        Eigen::Matrix2d T_tl;
        T_tl << grad, orth;
        Eigen::Matrix2d T_lt = T_tl.transpose();
        return T_lt; // track to local tangent
    }

    template<typename x_t, typename p_t,
            typename T = typename x_t::Scalar>
    T get_non_control_cost(const Eigen::MatrixBase<x_t> &x,
                           const Eigen::MatrixBase<p_t> &p,
                           const Scalar &tau)
    {
        const Eigen::Vector2<T> pos = x.template head<2>();
        unused(p);
        const unsigned i = std::ceil(tau * MS_N_SEGS);

        T non_control_cost{0};

        // Lookup pos ref
        const Eigen::Vector2d pos_ref = configParameters.raceline_refs.row(i).transpose();
        const Eigen::Vector2<T> pos_err = pos_ref - pos;

        const Eigen::Vector2d tangent = configParameters.grad_refs.row(i).transpose();
        const Eigen::Matrix2d T_lk = gradToTransMat(tangent.normalized());
        const Eigen::Vector2<T> t_pos_err = T_lk * pos_err;

        non_control_cost += t_pos_err.dot(tuningParameters.W_diag_path_tracking.asDiagonal() * t_pos_err);
//        non_control_cost += tuningParameters.W_diag_path_tracking(1) * t_pos_err(1) * t_pos_err(1); // lateral
//        non_control_cost += tuningParameters.W_diag_path_tracking(0) * t_pos_err(0) * t_pos_err(0); // longitudinal

//        non_control_cost += pos_err.dot(tuningParameters.W_diag_path_tracking.asDiagonal() * pos_err);

        if (USE_ACCEL)
        {
            const T speed = x(3);

            // Lookup speed ref
            const double speed_ref = track->get_info().length * configParameters.path_velocities(i);
            T speed_err = speed_ref - speed;
            non_control_cost += tuningParameters.w_speed_tracking * speed_err * speed_err;
        }
        return non_control_cost;
    }
    template<typename u_t, typename p_t,
            typename T = typename u_t::Scalar>
    T get_control_cost(const Eigen::MatrixBase<u_t> &u,
                       const Eigen::MatrixBase<p_t> &p,
                       const Scalar &tau)
    {
        unused(p);
        const unsigned i = std::ceil(tau * MS_N_SEGS);

        T control_cost = u.dot(tuningParameters.R_diag.head<2>().asDiagonal() * u);

        if (USE_ACCEL) {}
        else
        {
            const T speed = model.speed_gain * u(1);

            // Lookup speed ref
            const double speed_ref = spline.get_path_length() * configParameters.path_velocities(i);
            T speed_err = speed_ref - speed;
            control_cost += tuningParameters.w_speed_tracking * speed_err * speed_err;
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
        unused(t0, tf);
        return get_non_control_cost(x, p, tau) + get_control_cost(u, p, tau);
    }

    template<typename x_t, typename p_t, typename t0_t, typename tf_t,
            typename T = typename x_t::Scalar>
    T mayer_term_impl(const Eigen::MatrixBase<x_t> &xf,
                      const Eigen::MatrixBase<p_t> &p,
                      const Eigen::MatrixBase<t0_t> &t0,
                      const Eigen::MatrixBase<tf_t> &tf)
    {
        const Eigen::Vector2<T> posf = xf.template head<2>();
        const Eigen::Vector2<T> pos_err = configParameters.raceline_refs.row(MS_N_SEGS).transpose() - posf;
        unused(t0, tf);

        T mayer_cost{0};

        // mayer_cost += tuningParameters.mayer_factor * get_non_control_cost(xf, p, 1.0);
        mayer_cost += tuningParameters.mayer_factor * pos_err.dot(pos_err);

        if (USE_TRACKBOUNDS)
        {
            const T s_contour = p(0);
            mayer_cost += 1e3 * (s_contour * s_contour + s_contour);
        }
        return mayer_cost;
    }

    template<typename x_t, typename u_t, typename p_t,
            typename T = typename x_t::Scalar>
    state_t <T> dynamics_impl(const Eigen::MatrixBase<x_t> &x,
                              const Eigen::MatrixBase<u_t> &u,
                              const Eigen::MatrixBase<p_t> &p)
    {
        return model.template dynamics_impl<T>(x, u, p.template head<0>());
    }

    /* Inequality constraints */
    template<typename x_t, typename u_t, typename p_t,
            typename T = typename x_t::Scalar> // T is scalar type
    ineq_constr_t<T> inequality_constraints_impl(const Eigen::MatrixBase<x_t>& x,
                                                 const Eigen::MatrixBase<u_t>& u,
                                                 const Eigen::MatrixBase<p_t>& p,
                                                 const Scalar& tau)
    {
//        if (NG > 0)
//        {
//            std::cerr << "control_problem_base: NG = " << NG << " but inequality_constraints_impl() not implemented.\n";
//            exit(EXIT_FAILURE);
//        }
        const Eigen::Vector2<T> pos = x.template head<2>();
        const T s_contour = p(0);

        const unsigned i = std::ceil(tau * MS_N_SEGS);

        const Eigen::Vector2d pos_ref = configParameters.centerline_refs.row(i).transpose();
        const Eigen::Vector2<T> pos_err = pos_ref - pos;

        const Eigen::Vector2d tangent = configParameters.centerline_grad_refs.row(i).transpose();
        const Eigen::Matrix2d T_lk = gradToTransMat(tangent);
        const Eigen::Vector2<T> t_pos_err = T_lk * pos_err;

        ineq_constr_t<T> ineqConstr;
        if (USE_TRACKBOUNDS)
        {
            ineqConstr(0) = (t_pos_err(1) - 0.35 - s_contour);
            ineqConstr(1) = (-0.35 - t_pos_err(1) - s_contour);
        }
        return ineqConstr;
    }

protected:
    Eigen::VectorXd x0_epsi;
    bool solve_succeeded{false};

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

        const double V = 0.5;

        for (unsigned i = 0; i < N_points - 1; i++)
        {
            Eigen::VectorXd x = X_guess.col(i);
            Eigen::VectorXd x_next = x;

            // Integrate heading and position
            x_next(px) += (V * cos(x_next(thet))) * dt; // vx * cos(theta)
            x_next(py) += (V * sin(x_next(thet))) * dt; // vx * sin(theta)

            X_guess.col(i + 1) = x_next;
        }
        return X_guess;
    }
};

}
}
}

#endif //SRC_PATHTRACKINGLAOPTOCP_HPP
