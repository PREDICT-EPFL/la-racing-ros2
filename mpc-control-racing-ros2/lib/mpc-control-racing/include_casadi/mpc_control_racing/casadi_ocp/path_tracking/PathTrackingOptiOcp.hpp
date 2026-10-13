#ifndef SRC_PATHTRACKINGOPTIOCP
#define SRC_PATHTRACKINGOPTIOCP

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/casadi_model/kinematic_bicycle/SimpleBicycle.hpp"
#include "mpc_control_racing/casadi_model/kinematic_bicycle/AcceleratedBicycle.hpp"

#include "mpc_control_racing/problem/path_tracking/PathTrackingTranscriptionSettings.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following/CasadiSpline.hpp"

#include "mpc_control_racing/shift_warm_start_heading_casadi.hpp"
#include "utils_racing/PathUtils.hpp"

#define USE_U_CMD false
#define USE_ACCEL false

namespace mpc_control_racing {
namespace path_tracking {
namespace casadi_ocp {

#if USE_U_CMD
using Model = mpc_control_racing::casadi_model::SimpleBicycleWithActuators;
#elif USE_ACCEL
using Model = mpc_control_racing::casadi_model::AcceleratedBicycleMX;
#else
using Model = mpc_control_racing::casadi_model::SimpleBicycleMX;
#endif

class PathTrackingOptiOcp : public mpc_control_common::ocp_casadi::OptiOcp,
                            public mpc_control_racing::RacingOptimizerInterface
{
public:
    PathTrackingOptiOcp()
    {
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
        if (USE_U_CMD) { x_comp(7) = state.throttle; }

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
    }
    bool init() override
    {
        Model model;
//        DM x0 = DM::vertcat({0.510, -0.126, -0.207});
//        DM u = DM::vertcat({0.4, 0.2});
//        DM p = DM::vertcat({});
//        DM d = DM::vertcat({});
//        std::cout << "Model evaluation test " << model.dynamics(casadi::DMVector({x0, u, p, d}))[0].nonzeros() << "\n";
        /// Model is identical

        DM tangent = DM::vertcat({0.3, 0.1});
        std::cout << "tangent: " << tangent << "\nmat: " << gradToTransMat(tangent) << "\n";

        init_base(model.dynamics, horizon);

        /* Symbolic OptiStack parameters */
        x0_sym = opti->parameter(nx); // Symbolic initial state

        pos_refs_sym = opti->parameter(2, path_tracking::MS_N_SEGS + 1);
        grad_refs_sym = opti->parameter(2, path_tracking::MS_N_SEGS + 1);
        speed_refs_sym = opti->parameter(path_tracking::MS_N_SEGS + 1);
        centerline_refs_sym = opti->parameter(2, path_tracking::MS_N_SEGS + 1);
        centerline_grad_refs_sym = opti->parameter(2, path_tracking::MS_N_SEGS + 1);

        W_diag_path_tracking_sym = opti->parameter(2); // Symbolic path tracking weight
        w_speed_tracking_sym = opti->parameter(1);
        R_diag_sym = opti->parameter(nu); // Symbolic input weight diagonal
        R_du_diag_sym = opti->parameter(nu); // Symbolic input derivative weight diagonal
        mayer_factor_sym = opti->parameter(1); // Symbolic mayer factor

//        s_contour_sym = opti->variable(1);

        for (unsigned i = 0; i < MS_N_SEGS + 1; i++)
        {
            const std::vector<DM> pos_grad_hess_tgnt_curv = spline.spline_func(DM(configParameters.path_params(i)));
            configParameters.raceline_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[0]).transpose();
            configParameters.grad_refs.row(i) = eigen_casadi_conversion::toEigenX(pos_grad_hess_tgnt_curv[1]).transpose();
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
        spline.load_from_yaml(yaml_filepath, "pos", spline_following::casadi_ocp::CasadiSpline::Pos);
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
        for (unsigned i = 0; i < MS_N_SEGS + 1; i++) { configParameters.grad_refs.row(i).normalize(); }

//        int defaultPrecision = std::cout.precision();
//        std::cout << std::fixed << std::setprecision(3)
//                  << "tracking config: p " << configParameters.path_params.transpose() << "\n"
//                  << "P:\n" << configParameters.pos_refs.transpose() << "\n"
//                  << "G:\n" << configParameters.grad_refs.transpose() << "\n"
//                  << std::defaultfloat << std::setprecision(defaultPrecision);

        using namespace eigen_casadi_conversion;
        opti->set_value(pos_refs_sym, toDM(configParameters.raceline_refs.transpose()));
        opti->set_value(grad_refs_sym, toDM(configParameters.grad_refs.transpose()));
        opti->set_value(speed_refs_sym, toDM(track->get_info().length * configParameters.path_velocities));
        opti->set_value(centerline_refs_sym, toDM(configParameters.centerline_refs.transpose()));
        opti->set_value(centerline_grad_refs_sym, toDM(configParameters.centerline_grad_refs.transpose()));

        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
        opti->set_value(W_diag_path_tracking_sym, eigen_casadi_conversion::toDM(tuningParameters.W_diag_path_tracking));
        opti->set_value(w_speed_tracking_sym, tuningParameters.w_speed_tracking);

        opti->set_value(R_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_diag.topRows(2)));
        opti->set_value(R_du_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_du_diag.topRows(2)));

        opti->set_value(mayer_factor_sym, tuningParameters.mayer_factor);

        tuningParameters.needs_update = false;
    }

    /* Problem definition */
    void setup_discretized_trajectory_constraints() override
    {
        casadi::Slice all;

        // Box constraints on input
        set_ubu(casadi::DM::vertcat({1, tuningParameters.throttle_max})); // Steering, throttle
        set_lbu(casadi::DM::vertcat({-1, tuningParameters.throttle_min}));

        // Move blocking on first couple inputs
        const unsigned join_n_inputs = 1; // 1 means no move blocking
        for (unsigned k = 0; k < join_n_inputs - 1; k++)
        {
            std::cout << "Constraint: input " << k << " == input " << k + 1 << "\n";
            sym_t du = (U_sym(all, k + 1) - U_sym(all, k));
//                opti->subject_to(U_sym(all, k) == U_sym(all, k + 1));
            opti->minimize(opti->f() + 1000 * sym_t::mtimes(du.T(), du));
        }

        /* Differential constraint on input */
        for (unsigned iu = 0; iu < U_sym.size1(); iu++)
        {
            if (tuningParameters.R_du_diag(iu) > 0)
            {
                std::cout << "Cost on discrete input differences for input " << iu << "\n";
                for (unsigned k = 1; k < U_sym.size2(); k++)
                {
                    sym_t du = (U_sym(all, k) - U_sym(all, k - 1));
                    opti->minimize(opti->f() + R_du_diag_sym(iu) * sym_t::sq(du(iu)));
                }
            }
        }

        // Boundary conditions: Initial state (X, Y, theta)
        casadi::MX epsi;
        if (USE_ACCEL) { epsi = casadi::DM::vertcat({0.001, 0.001, 0.002, 0.001}); }
        else if (USE_U_CMD) { epsi = casadi::DM::vertcat({0.001, 0.001, 0.002, 0.1, 0.1}); }
        else { epsi = casadi::DM::vertcat({0.001, 0.001, 0.002}); }
        opti->subject_to(x0_sym - epsi <= X_sym(all, 0) <= x0_sym + epsi);

        // Boundary conditions: Final state

        // One slack variable for all contouring constraints
//        opti->subject_to({0 <= s_contour_sym <= 0.5});
//        sym_t slack_penalty = 1e3 * (s_contour_sym * s_contour_sym + s_contour_sym);
//        opti->minimize(opti->f() + slack_penalty);
    }

    static sym_t gradToTransMat(sym_t grad)
    {
        // Return transformation matrix from track frame to local tangent frame
        sym_t orth = sym_t::vertcat({-grad(1), grad(0)});
        sym_t T_tl = sym_t::horzcat({grad, orth});
        sym_t T_lt = T_tl.T();
        return T_lt; // track to local tangent
    }

    sym_t get_non_control_cost(const sym_t &x, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        sym_t pos = x(casadi::Slice(0, 2));
        (void) p;
        (void) d;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t non_control_cost = 0;

        // Lookup pos ref
        //std::cout << "t: " << t << "-> stage " << i << "\n";
        sym_t pos_ref = pos_refs_sym(casadi::Slice(), i);
        sym_t pos_err = pos_ref - pos;

        sym_t tangent = grad_refs_sym(casadi::Slice(), i);
        sym_t T_lk = gradToTransMat(tangent);
        sym_t t_pos_err = sym_t::mtimes(T_lk, pos_err);

        non_control_cost +=
                sym_t::mtimes(t_pos_err.T(), sym_t::mtimes(sym_t::diag(W_diag_path_tracking_sym), t_pos_err));
//        non_control_cost += W_diag_path_tracking_sym(1) * sym_t::sq(t_pos_err(1)); // lateral
//        non_control_cost += W_diag_path_tracking_sym(0) * sym_t::sq(t_pos_err(0)); // longitudinal

//        non_control_cost += sym_t::mtimes(pos_err.T(), sym_t::mtimes(sym_t::diag(W_diag_path_tracking_sym), pos_err));

        if (USE_ACCEL)
        {
            sym_t speed = x(3);

            // Lookup speed ref
            sym_t speed_ref = speed_refs_sym(i);
            sym_t speed_err = speed_ref - speed;
            non_control_cost += w_speed_tracking_sym * sym_t::sq(speed_err);
        }
        return non_control_cost;
    }
    sym_t get_control_cost(const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        (void) p;
        (void) d;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t control_cost = 0;

        control_cost += sym_t::mtimes(u.T(), sym_t::mtimes(sym_t::diag(R_diag_sym), u));

        if (USE_ACCEL) {}
        else
        {
            sym_t speed = casadi_model::SimpleBicycleMX::speed_gain * u(1);

            // Lookup speed ref
            sym_t speed_ref = speed_refs_sym(i);
            sym_t speed_err = speed_ref - speed;
            control_cost += w_speed_tracking_sym * sym_t::sq(speed_err);
        }
        return control_cost;
    }

    sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        (void) p;
        (void) d;
        return get_non_control_cost(x, p, d, t) + get_control_cost(u, p, d, t);
    }
    sym_t mayer_cost_impl(const sym_t &xf, const sym_t &p, const sym_t &d, const sym_t &tf) override
    {
        sym_t posf = xf(casadi::Slice(0, 2));
        sym_t pos_err = pos_refs_sym(casadi::Slice(), MS_N_SEGS) - posf;
        (void) p;
        (void) d;
//        return mayer_factor_sym * get_non_control_cost(xf, p, d, tf); // + sym_t::mtimes(pos_err.T(), pos_err);
        return mayer_factor_sym * sym_t::mtimes(pos_err.T(), pos_err);
//        return 0;
    }
    sym_t cont_dynamics_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d) override
    {
        using symvec_t = std::vector<sym_t>;
        return plant_dynamics(symvec_t({x, u, p, d}))[0];
    }
    sym_t eq_constraints_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        (void) x;
        (void) u;
        (void) p;
        (void) d;
        (void) t;

        return {};
    }
    sym_t ineq_constraints_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        sym_t pos = x(casadi::Slice(0, 2));
        (void) u;
        (void) p;
        (void) d;
        (void) t; // unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t ineq_constr = {};

//        if (i <= 4)
        {
            /* Contouring constraints (local linear approximations) */
//            sym_t pos_ref = centerline_refs_sym(casadi::Slice(), i);
//            sym_t pos_err = pos_ref - pos;
//
//            sym_t tangent = centerline_grad_refs_sym(casadi::Slice(), i);
//            sym_t T_lk = gradToTransMat(tangent);
//            sym_t t_pos_err = sym_t::mtimes(T_lk, pos_err);
//
//            ineq_constr = sym_t::vertcat({ineq_constr,
//                                          t_pos_err(1) - 0.35 - s_contour_sym,
//                                          -0.35 - t_pos_err(1) - s_contour_sym
//                                         });
        }
        return ineq_constr;
    }

protected:
    sym_t x0_sym;
    sym_t pos_refs_sym;
    sym_t grad_refs_sym;
    sym_t speed_refs_sym;
    sym_t centerline_refs_sym;
    sym_t centerline_grad_refs_sym;

    sym_t W_diag_path_tracking_sym;
    sym_t w_speed_tracking_sym;
    sym_t R_diag_sym;
    sym_t R_du_diag_sym;
    sym_t mayer_factor_sym;

    sym_t s_contour_sym;

    void shift_heading()
    {
        if (solution_avail)
        {
            DM heading_warm = X_opt(2, casadi::Slice());
            if (mpc_control_racing::casadi_ocp::shift_warm_heading(state.turn_angle, heading_warm))
            {
                opti->set_initial(X_sym(2, casadi::Slice()), heading_warm);
            }
        }
    }

private:
//    casadi::Function spline = spline_following::casadi_ocp::LA_track_spline();
    spline_following::casadi_ocp::CasadiSpline spline;
    utils_racing::PathLocator track_locator;
};

}
}
}

#endif //SRC_PATHTRACKINGOPTIOCP
