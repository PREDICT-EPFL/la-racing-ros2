#ifndef SRC_PATHTRACKINGTIREOPTIOCP
#define SRC_PATHTRACKINGTIREOPTIOCP

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/casadi_model/dynamic_bicycle/DynamicBicycle.hpp"
#include "mpc_control_racing/casadi_model/dynamic_bicycle/DynamicBicycleWithActuators.hpp"

#include "mpc_control_racing/problem/path_tracking/PathTrackingTranscriptionSettings.hpp"

#include "mpc_control_racing/shift_warm_start_heading_casadi.hpp"
#include "utils_racing/PathUtils.hpp"

#define USE_U_DOT false
#define USE_U_CMD false

namespace mpc_control_racing {
namespace path_tracking {
namespace casadi_ocp {

class PathTrackingTireOptiOcp : public mpc_control_common::ocp_casadi::OptiOcp,
                                public mpc_control_racing::RacingOptimizerInterface
{
public:
    PathTrackingTireOptiOcp()
    {
        /* Define state */
        state_idx.pos_x = 0;
        state_idx.pos_y = 1;
        state_idx.turn_angle = 2;
        state_idx.vel_x = 3;
        state_idx.vel_y = 4;
        state_idx.turn_rate = 5;
        if (USE_U_DOT || USE_U_CMD)
        {
            state_idx.steering = 6;
            state_idx.throttle = 7;
        }

        /* Initialize state */
        state.pos = {0, 0};
        state.turn_angle = 0;
        state.vel = {0.2, 0.0};
        state.turn_angle = 0.0;
        if (USE_U_CMD)
        {
            state.steering = 0;
            state.throttle = 0.2;
        }

        if (USE_U_DOT || USE_U_CMD) { x_comp.resize(6 + 2); }
        else { x_comp.resize(6); }
        x_comp.setZero();
        x_comp(state_idx.vel_x) = state.vel(0);
        if (USE_U_DOT || USE_U_CMD) { x_comp(7) = state.throttle; }

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

        /* Tuning parameters */
        tuningParameters.W_diag_path_tracking << 1, 1;
        tuningParameters.w_speed_tracking = 1;
        tuningParameters.R_diag.topRows(2) << 1, 1;
        tuningParameters.mayer_factor = 10;
    }
    bool init() override
    {
        if (USE_U_CMD)
        {
            casadi_model::DynamicBicycleWithActuatorsMX model;
            init_base(model.dynamics, horizon);
//            DM x0 = DM::vertcat({0.510, -0.126, -0.207, 0.501, -0.001, -0.001, 0.3, 0.15});
//            DM u = DM::vertcat({0.4, 0.2});
//            DM p = DM::vertcat({});
//            DM d = DM::vertcat({});
//            std::cout << "Model evaluation test " << model.dynamics(casadi::DMVector({x0, u, p, d}))[0].nonzeros() << "\n";
            // Model is identical
        }
        else
        {
            casadi_model::DynamicBicycleMX model;
            init_base(model.dynamics, horizon);
        }

        /* Symbolic OptiStack parameters */
        x0_sym = opti->parameter(nx); // Symbolic initial state

        pos_refs_sym = opti->parameter(2, MS_N_SEGS + 1);
        grad_refs_sym = opti->parameter(2, MS_N_SEGS + 1);
        speed_refs_sym = opti->parameter(MS_N_SEGS + 1);
        centerline_refs_sym = opti->parameter(2, MS_N_SEGS + 1);
        centerline_grad_refs_sym = opti->parameter(2, MS_N_SEGS + 1);

        W_diag_path_tracking_sym = opti->parameter(2); // Symbolic path tracking weight
        W_diag_v_body_sym = opti->parameter(2); // Symbolic body vel weight
        w_speed_tracking_sym = opti->parameter(1);
        w_turn_rate_sym = opti->parameter(1);
        R_diag_sym = opti->parameter(nu); // Symbolic input weight diagonal
        R_du_diag_sym = opti->parameter(nu); // Symbolic input derivative weight diagonal
        mayer_factor_sym = opti->parameter(1); // Symbolic mayer factor

        for (unsigned i = 0; (i < MS_N_SEGS + 1); i++)
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
        tuningParameters.w_turn_rate = yamlNode["w_turn_rate"].as<double>();

        const std::vector<double> W_diag_v_body = yamlNode["W_diag_v_body"].as<std::vector<double>>();
        tuningParameters.W_diag_v_body << W_diag_v_body[0], W_diag_v_body[1];

        const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Path Tracking Tire OCP -- \n"
                  << "W_diag_path_tracking: " << tuningParameters.W_diag_path_tracking.transpose() << "\n"
                  << "W_diag_v_body: " << tuningParameters.W_diag_v_body.transpose() << "\n"
                  << "w_speed_tracking: " << tuningParameters.w_speed_tracking << "\n"
                  << "w_turn_rate: " << tuningParameters.w_turn_rate << "\n"
                  << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath){

        spline.load_from_yaml(yaml_filepath, "pos", spline_following::casadi_ocp::CasadiSpline::Pos);
        init();
    }

    /* Updaters for state and parameters */
    Eigen::VectorXd compose_state() override
    {
        Eigen::Matrix2d T_body_world;
        T_body_world << std::cos(state.turn_angle), std::sin(state.turn_angle),
                -std::sin(state.turn_angle), std::cos(state.turn_angle);

        Eigen::Vector2d vel_body = T_body_world * state.vel;
        // std::cout << "vel_body: " << vel_body << std::endl;
        vel_body(0) = std::max(0.5, vel_body(0)); // Project vx onto speed range (lower bound)
        vel_body(1) = std::max(-0.049, std::min(vel_body(1), 0.049)); // Project vy

        // Assemble state from available measurements in state struct
        if (USE_U_DOT || USE_U_CMD)
        {
            if (OptiOcp::solve_succeeded)
            {
                // Use previously planned control as initial control for control evolution
                state.steering = get_control_trajectory()(0, 1);
                state.throttle = get_control_trajectory()(1, 1);
            }
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate,
                    state.steering, state.throttle;
        }
        else
        {
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate;
        }
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
        opti->set_value(speed_refs_sym, toDM(track->get_info().length * configParameters.path_params));
        opti->set_value(centerline_refs_sym, toDM(configParameters.centerline_refs.transpose()));
        opti->set_value(centerline_grad_refs_sym, toDM(configParameters.centerline_grad_refs.transpose()));

        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
        opti->set_value(W_diag_path_tracking_sym, eigen_casadi_conversion::toDM(tuningParameters.W_diag_path_tracking));
        opti->set_value(W_diag_v_body_sym, eigen_casadi_conversion::toDM(tuningParameters.W_diag_v_body));
        opti->set_value(w_speed_tracking_sym, tuningParameters.w_speed_tracking);
        opti->set_value(w_turn_rate_sym, tuningParameters.w_turn_rate);

        opti->set_value(R_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_diag.topRows(2)));
        opti->set_value(R_du_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_du_diag.topRows(2)));

        opti->set_value(mayer_factor_sym, tuningParameters.mayer_factor);

        tuningParameters.needs_update = false;
    }

    /* Problem definition */
    void setup_discretized_trajectory_constraints() override
    {
        Slice all;

        if (USE_U_DOT)
        {
            // Box constraints on input (in state)
            set_ubu(DM::vertcat({0.5, 10}));
            set_lbu(DM::vertcat({-0.5, 10}));

            opti->subject_to(-1 <= X_sym(state_idx.steering, all) <= 1);
            opti->subject_to(tuningParameters.throttle_min <= X_sym(state_idx.throttle, all) <= tuningParameters.throttle_max);
        }
        else // or USE_U_CMD
        {
            // Box constraints on input
            set_ubu(DM::vertcat({1, tuningParameters.throttle_max}));
            set_lbu(DM::vertcat({-1, tuningParameters.throttle_min}));
            // To update throttle_max based on manual cap, need to use throttle_max_sym and update throttle_max_sym
            // in update_config_params()
//            opti->subject_to(-1 <= U_sym(0, all) <= 1);
//            opti->subject_to(throttle_min_sym <= U_sym(1, all) <= throttle_max_sym);

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
        }

        // Box constraints on state
        opti->subject_to(0.2 <= X_sym(state_idx.vel_x, all)); // 0.1 < vx // Redundant to lower throttle limit
//        opti->subject_to(-0.05 <= X_sym(state_idx.vel_y, all) <= 0.05); // vy

        // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega)
//        casadi::MX epsi = casadi::DM::vertcat({0.005, 0.005, 0.005, 0.1, 0.005, 0.005});
        casadi::MX epsi = casadi::DM::vertcat({0.005, 0.005, 0.005, 0.1, 0.005, 0.005});
        if (USE_U_CMD) epsi = casadi::MX::vertcat({epsi, 0.1, 0.1});
        opti->subject_to(x0_sym - epsi <= X_sym(all, 0) <= x0_sym + epsi);

        // Boundary conditions: Final state

        std::cout << "No constraints: " << opti->g().size() << "\n";
    }

    static sym_t gradToTransMat(sym_t grad)
    {
        sym_t orth = sym_t::vertcat({-grad(1), grad(0)});
        sym_t T_lt = sym_t::horzcat({grad, orth}).T();
        return T_lt;
    }

    sym_t get_non_control_cost(const sym_t &x, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        sym_t pos = x(Slice(0, state_idx.pos_y + 1));
        (void) p;
        (void) d;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t non_control_cost = 0;

        // Velocity states
//        sym_t xv_ref = DM::vertcat({1.5, 0, 0});
//        sym_t Qvdiag = sym_t::vertcat({W_diag_v_body_sym, w_turn_rate_sym});

//        sym_t xv_err = xv_ref - x(Slice(3, 6));
//        non_control_cost += sym_t::mtimes(xv_err.T(),
//                                          sym_t::mtimes(sym_t::diag(Qvdiag), xv_err));

        // Lookup pos ref
        //std::cout << "t: " << t << "-> stage " << i << "\n";
        sym_t pos_ref = pos_refs_sym(Slice(), i);
        sym_t pos_err = pos_ref - pos;

        sym_t tangent = grad_refs_sym(Slice(), i);
        sym_t T_lk = gradToTransMat(tangent);
        sym_t t_pos_err = sym_t::mtimes(T_lk, pos_err);

        non_control_cost +=
                sym_t::mtimes(t_pos_err.T(), sym_t::mtimes(sym_t::diag(W_diag_path_tracking_sym), t_pos_err));
//        non_control_cost += W_diag_path_tracking_sym(1) * sym_t::sq(t_pos_err(1)); // lateral
//        non_control_cost += W_diag_path_tracking_sym(0) * sym_t::sq(t_pos_err(0)); // longitudinal

//        non_control_cost += sym_t::mtimes(pos_err.T(), sym_t::mtimes(sym_t::diag(W_diag_path_tracking_sym), pos_err));

        sym_t turn_rate = x(state_idx.turn_rate);
        non_control_cost += w_turn_rate_sym * sym_t::sq(turn_rate);

//        sym_t vy = x(state_idx.vel_y);
//        non_control_cost += W_diag_v_body_sym(1) * sym_t::sq(vy);

        if (USE_U_DOT)
        {
            sym_t u_in_state = x(Slice(6, 8));
            non_control_cost += sym_t::mtimes(u_in_state.T(), sym_t::mtimes(sym_t::diag(R_diag_sym), u_in_state));
        }
        return non_control_cost;
    }
    sym_t get_control_cost(const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        sym_t speed = casadi_model::DynamicBicycleMX::speed_gain * u(1);
        (void) p;
        (void) d;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t control_cost = 0;

        if (USE_U_DOT) control_cost += sym_t::mtimes(u.T(), sym_t::mtimes(sym_t::diag(R_du_diag_sym), u));
        else control_cost += sym_t::mtimes(u.T(), sym_t::mtimes(sym_t::diag(R_diag_sym), u));

        // Lookup speed ref
        sym_t speed_ref = speed_refs_sym(i);
        sym_t speed_err = speed_ref - speed;
        control_cost += w_speed_tracking_sym * sym_t::sq(speed_err);
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
        sym_t posf = xf(Slice(0, state_idx.pos_y + 1));
        sym_t pos_err = pos_refs_sym(Slice(), MS_N_SEGS) - posf;
        (void) p;
        (void) d;
//        return mayer_factor_sym * get_non_control_cost(xf, p, d, tf); // + sym_t::mtimes(pos_err.T(), pos_err);
//        return mayer_factor_sym * sym_t::mtimes(pos_err.T(), pos_err); // Results in snake-like trajectories around the reference position if those don't progress enough
        return 0;
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
        sym_t pos = x(Slice(0, 2));
        (void) u;
        (void) p;
        (void) d;
        (void) t; //unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t ineq_constr = {};

//        if (i > 4)
        {
//            /* Contouring constraints (local linear approximations) */
//            sym_t pos_ref = centerline_refs_sym(Slice(), i);
//            sym_t pos_err = pos_ref - pos;
//
//            sym_t tangent = centerline_grad_refs_sym(Slice(), i);
//            sym_t T_lk = gradToTransMat(tangent);
//            sym_t t_pos_err = sym_t::mtimes(T_lk, pos_err);
//
//            ineq_constr = sym_t::vertcat({ineq_constr,
//                                          t_pos_err(1) - 0.35,
//                                          -0.35 - t_pos_err(1)
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
    sym_t W_diag_v_body_sym;
    sym_t w_speed_tracking_sym;
    sym_t w_turn_rate_sym;
    sym_t R_diag_sym;
    sym_t R_du_diag_sym;
    sym_t mayer_factor_sym;

    bool shift_heading()
    {
        bool modified{false};
        if (solution_avail)
        {
            DM heading_warm = X_opt(state_idx.turn_angle, Slice());
            modified = (mpc_control_racing::casadi_ocp::shift_warm_heading(state.turn_angle, heading_warm));
            if (modified) { opti->set_initial(X_sym(state_idx.turn_angle, Slice()), heading_warm); }
        }
        return modified;
    }

    DM generate_X_guess(const DM &x0, const double dt, unsigned N_points)
    {
        DM X_guess = DM::repmat(x0, 1, N_points);
        int px = state_idx.pos_x;
        int py = state_idx.pos_y;
        int thet = state_idx.turn_angle;
        int vx = state_idx.vel_x;
        int vy = state_idx.vel_y;
        int w = state_idx.turn_rate;

        for (unsigned i = 0; i < N_points - 1; i++)
        {
            DM x = X_guess(Slice(), i);
            DM x_next = x;

            // Integrate heading and position
            x_next(thet) += x(w) * dt;
            x_next(px) += (x(vx) * cos(x_next(thet)) - x(vy) * sin(x_next(thet))) * dt; // vx * cos(theta) - vy * sin(theta)
            x_next(py) += (x(vx) * sin(x_next(thet)) + x(vy) * cos(x_next(thet))) * dt; // vx * sin(theta) + vy * cos(theta)

            //x_next(6) += x(7) * dt; // Integrate path param velocity

            X_guess(Slice(), i + 1) = x_next;
        }
        return X_guess;
    }

private:
    using Slice = casadi::Slice;
//    casadi::Function spline = spline_following::casadi_ocp::LA_track_spline();
    spline_following::casadi_ocp::CasadiSpline spline;
    utils_racing::PathLocator track_locator;
};

}
}
}

#endif //SRC_PATHTRACKINGTIREOPTIOCP
