#ifndef SRC_SPLINEFOLLOWINGOPTIOCP
#define SRC_SPLINEFOLLOWINGOPTIOCP

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/casadi_model/StackedSystem.hpp"
#include "mpc_control_racing/casadi_model/kinematic_bicycle/SimpleBicycle.hpp"
#include "mpc_control_racing/casadi_model/AcceleratedPathparam.hpp"

#include "mpc_control_racing/casadi_ocp/spline_following/CasadiSpline.hpp"

#include "mpc_control_racing/shift_warm_start_heading_casadi.hpp"
#include "utils_racing/PathUtils.hpp"

#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#define USE_TRACKBOUNDS true

namespace mpc_control_racing {
namespace spline_following {
namespace casadi_ocp {

using namespace casadi_model;

using SimpleBicycleWithTrackingMX = StackedSystem<SimpleBicycle<casadi::MX>, AcceleratedPathparam<casadi::MX>>;

class SplineFollowingOptiOcp : public mpc_control_common::ocp_casadi::OptiOcp,
                               public mpc_control_racing::RacingOptimizerInterface
{
public:
    SplineFollowingOptiOcp()
    {
        /* Define state */
        state_idx.pos_x = 0;
        state_idx.pos_y = 1;
        state_idx.turn_angle = 2;
        state_idx.path_param = 3;
        state_idx.path_param_vel = 4;

        /* Initialize state */
        state.pos = {0, 0};
        state.turn_angle = 0;
        state.path_param = 0;
        state.path_param_vel = 0;

        x_comp.resize(5);
//        x_comp.resize(4);
//        x_comp.resize(6); // augment by steering angle
        x_comp.setZero();

        /* Tuning parameters */
        tuningParameters.w_path_tracking = 10;
        tuningParameters.w_path_progression = 4;
        tuningParameters.R_diag = {1, 1, 1};
        tuningParameters.mayer_factor = 10;
    }
    bool init() override
    {
        SimpleBicycleWithTrackingMX model;
//        DM x0 = DM::vertcat({0.510, -0.126, -0.207, 0.3, 0.15});
//        DM u = DM::vertcat({0.4, 0.2, 0.05});
//        DM p = DM::vertcat({});
//        DM d = DM::vertcat({});
//        std::cout << "Model evaluation test " << model.dynamics(casadi::DMVector({x0, u, p, d}))[0].nonzeros() << "\n";
        /// Model is identical
        init_base(model.dynamics, horizon);

        /* Symbolic problem parameters */
        x0_sym = opti->parameter(nx); // Symbolic initial state
        gamma0_sym = opti->parameter(1);

        w_path_progression_sym = opti->parameter(1); // Symbolic path progression weight
        w_path_tracking_sym = opti->parameter(1); // Symbolic path tracking weight
        R_diag_sym = opti->parameter(nu); // Symbolic input weight diagonal
        mayer_factor_sym = opti->parameter(1); // Symbolic mayer factor

//        lat_acc_max_sym = opti->parameter(1); // Maximum lateral acceleration
//        s_lat_acc_sym = opti->variable(1); // Slack variable for lateral acceleration constraint

//        s_contour_sym = opti->variable(1);

        update_config_params();
        update_tuning_params();
//        std::cout << "Parameters after init():\n"
//                  << horizon << ", "
//                  << opti->value(w_path_tracking_sym) << ", "
//                  << opti->value(w_path_progression_sym) << ", "
//                  << opti->value(R_diag_sym) << ", "
//                  << opti->value(mayer_factor_sym) << "\n";
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

        tuningParameters.w_slack_contour = yamlNode["w_slack_contour"].as<double>();

        const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Spline Following OCP -- \n"
                  << "w_path_tracking: " << tuningParameters.w_path_tracking << "\n"
                  << "w_path_progression: " << tuningParameters.w_path_progression << "\n"
                  << "w_slack_contour: " << tuningParameters.w_slack_contour << "\n"
                  << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        path_spline.load_from_yaml(yaml_filepath, "pos", CasadiSpline::SplineType::Pos);
        offsLR_spline.load_from_yaml(yaml_filepath, "offsLR", CasadiSpline::SplineType::Generic);
    }

    /* Updaters for state and parameters */
    Eigen::VectorXd compose_state() override
    {
        state.path_param = track_locator.grid_search(track, state.pos, state.path_param);

//        double steering_angle{0};
//        if (sol) { steering_angle = sol->value(X_sym(0, 5)).nonzeros()[0]; }
//        x_comp << state.pos, state.turn_angle, state.path_param, state.path_param_vel, steering_angle;

        DM grad = path_spline.spline_func(DM(state.path_param))[1];
        grad = grad / DM::norm_2(grad);
        DM orth = DM::vertcat({-grad(1), grad(0)});
        DM T_lk = DM::horzcat({grad, orth}).T();
        DM l_vel = DM::mtimes(T_lk, eigen_casadi_conversion::toDM(state.vel));

        state.path_param_vel = l_vel(0).nonzeros()[0] / track->get_info().length;

        // Assemble state from available measurements in state struct
        x_comp << state.pos, state.turn_angle, state.path_param, state.path_param_vel;
//        x_comp << state.pos, state.turn_angle, state.path_param;;

        // opti->set_value(throttle_max_profile_sym, throttle_max_profile);

        return x_comp;
    }
    void update_config_params() override
    {
//        opti->set_value("", configParameters.V_ref);

        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        std::cout << "Update tuning params\n";
        opti->set_value(w_path_progression_sym, tuningParameters.w_path_progression);
        opti->set_value(w_path_tracking_sym, tuningParameters.w_path_tracking);
        opti->set_value(R_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_diag));
//        opti->set_value(R_du_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_du_diag.topRows(2)));

        opti->set_value(mayer_factor_sym, tuningParameters.mayer_factor);

//        opti->set_value(lat_acc_max_sym, tuningParameters.lat_acc_max);

        tuningParameters.needs_update = false;
    }

    /* Problem definition */
    void setup_discretized_trajectory_constraints() override
    {
        // Time-varying / parameterized max throttle
        // throttle_max_profile_sym = opti->parameter(1, U_sym.size2());
        // const sym_t UBU = sym_t::vertcat({
        //     1.0 * DM::ones(1, U_sym.size2()),
        //     throttle_max_profile_sym,
        //     1.0 * DM::ones(1, U_sym.size2())
        // });
        // set_ubu(UBU);

        // Box constraints on input
//        set_ubu(casadi::DM::vertcat(
//                {1, tuningParameters.throttle_max, 1})); // Steering, throttle, path parameter acceleration
//        set_lbu(casadi::DM::vertcat({-1, tuningParameters.throttle_min, -1}));

        // Box constraints on state
//        casadi::Slice all;
//        opti->subject_to(0 <= X_sym(3, all) <= 2); // Path parameter can only be in (positive) path range
//        opti->subject_to(0 < X_sym(4, all) <= 1); // Path velocity positive

//        for (unsigned i = 0; i < U_sym.size2(); i++)
//        {
//            /* Constraint on lateral acceleration */
//            sym_t steering = U_sym(0, i);
//            sym_t V_cmd = U_sym(1, i);
//            opti->subject_to(-lat_acc_max_sym < sym_t::sq(V_cmd) * steering < lat_acc_max_sym);
//
//            /* Differential constraint on steering */
//            if (i < U_sym.size2() - 1)
//            {
//                sym_t dSteering = (U_sym(0, i + 1) - U_sym(0, i));
//                opti->minimize(opti->f() + 1e-2 * dSteering * dSteering);
//            }
//        }

        // Move blocking on first couple inputs
//        const unsigned join_n_inputs = 1; // 1 means no move blocking
//        for (unsigned k = 0; k < join_n_inputs - 1; k++)
//        {
//            std::cout << "Constraint (or high cost): input " << k << " == input " << k + 1 << "\n";
//            sym_t du = (U_sym(all, k + 1) - U_sym(all, k));
////            opti->subject_to(du == 0);
//            opti->minimize(opti->f() + 1000 * sym_t::mtimes(du.T(), du));
//        }

        /* Differential constraint on input */
//        for (unsigned iu = 0; iu < U_sym.size1(); iu++)
//        {
//            if (tuningParameters.R_du_diag(iu) > 0)
//            {
//                std::cout << "Cost on discrete input differences for input " << iu << "\n";
//                for (unsigned k = 1; k < U_sym.size2(); k++)
//                {
//                    sym_t du = (U_sym(all, k) - U_sym(all, k - 1));
//                    opti->minimize(opti->f() + tuningParameters.R_du_diag(iu) * sym_t::sq(du(iu)));
//                }
//            }
//        }

        // Boundary conditions: Initial state (X, Y, theta, path, path vel)
//        casadi::MX epsi = casadi::DM::vertcat({0.005, 0.005, 0.005, 0.01, 0.5});
//        casadi::MX epsi = casadi::DM::vertcat({0.001, 0.001, 0.002, 0.01, 0.5});
//        opti->subject_to(x0_sym - epsi <= X_sym(all, 0) <= x0_sym + epsi);

        // Boundary conditions: Final state

        // One slack variable for all lateral acceleration constraints
//        opti->subject_to({0 <= s_lat_acc_sym <= 0.5});
//        sym_t slack_penalty = 1e3 * (s_lat_acc_sym * s_lat_acc_sym + s_lat_acc_sym);
//        opti->minimize(opti->f() + slack_penalty);'

        // One slack variable for all contouring constraints
//        opti->subject_to({0 <= s_contour_sym <= 999});
//        sym_t slack_penalty = tuningParameters.w_slack_contour * (s_contour_sym * s_contour_sym + s_contour_sym);
//        opti->minimize(opti->f() + slack_penalty);
        }

    static sym_t gradToTransMat(const sym_t &grad)
    {
        // Return transformation matrix from track frame to local tangent frame
        sym_t orth = sym_t::vertcat({-grad(1), grad(0)});
        sym_t T_tl = sym_t::horzcat({grad, orth});
        sym_t T_lt = T_tl.T();
        return T_lt; // track to local tangent
    }
    static sym_t get_Tz(const sym_t &angle)
    {
        sym_t Tz = sym_t::vertcat({sym_t::horzcat({sym_t::cos(angle), sym_t::sin(angle)}),
                                   sym_t::horzcat({-sym_t::sin(angle), sym_t::cos(angle)})});
        return Tz;
    }

    sym_t get_non_control_cost(const sym_t &x, const sym_t &p, const sym_t &d)
    {
        (void) p;
        (void) d;

        sym_t non_control_cost = 0;

        sym_t pos = x(casadi::Slice(0, 2));
        sym_t gamma = x(3);

        sym_t pos_ref = path_spline.spline_func(gamma)[0];
        sym_t pos_err = pos_ref - pos;
        non_control_cost += w_path_tracking_sym * sym_t::mtimes(pos_err.T(), pos_err);

        non_control_cost += -w_path_progression_sym * (gamma - gamma0_sym);

        return non_control_cost;
    }
    sym_t get_control_cost(const sym_t &u, const sym_t &p, const sym_t &d)
    {
        (void) p;
        (void) d;

        sym_t control_cost = sym_t::mtimes(u.T(), sym_t::mtimes(sym_t::diag(R_diag_sym), u));
        return control_cost;
    }

    sym_t lagrange_cost_impl(const sym_t &x, const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        (void) t;

        /* Track maximum possible speed */
//        sym_t gamma = x(3);
//        sym_t curvature = spline.spline_func(gamma)[4];
//        sym_t Vmax2 = lat_acc_max_sym / curvature;
//
//        sym_t Vcmd = u(1);
//        sym_t Verr = Vmax2 - Vcmd;
//        sym_t costV = sym_t::sq(Verr);

        return get_non_control_cost(x, p, d) + get_control_cost(u, p, d); //+ costV;
    }
    sym_t mayer_cost_impl(const sym_t &xf, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        (void) p;
        (void) d;
        (void) t;
//        return -w_path_progression_sym * xf(3);

        sym_t gamma_f = xf(3);
        return mayer_factor_sym * get_non_control_cost(xf, p, d);
        //       - w_path_progression_sym * (gamma_f - gamma0_sym); // Progression already in non_control cost
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
        sym_t gamma = x(3);
        (void) p;
        (void) d;
        (void) t;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS))); // #todo

        sym_t ineq_constr = {};

        // Boundary conditions: Initial state (X, Y, theta, path, path vel)
        if (i == 0)
        {
            casadi::MX epsi = casadi::DM::vertcat({0.005, 0.005, 0.005, 0.05, 0.01});
            ineq_constr = sym_t::vertcat({ineq_constr,
                                          (x0_sym - epsi) - x, x - (x0_sym + epsi)
                                         });
        }

        // Box constraints on state
        ineq_constr = sym_t::vertcat({ineq_constr,
                                      0 - x(state_idx.path_param), x(state_idx.path_param) - 2,
                                      0 - x(state_idx.path_param_vel), x(state_idx.path_param_vel) - 1,
                                      });

        // Box constraints on input
        DM ubu = DM::vertcat({1, tuningParameters.throttle_max, 1}); // Steering, throttle, path parameter acceleration
        DM lbu = DM::vertcat({-1, tuningParameters.throttle_min, -1});
        ineq_constr = sym_t::vertcat({ineq_constr,
                                      lbu - u, u - ubu});

//        sym_t curvature = path_spline.spline_func(gamma)[4];
//        sym_t Vcmd = 3.0 * u(1);

//        ineq_constr = sym_t::vertcat({ineq_constr,
//                                      Vcmd * Vcmd * abs(curvature) - lat_acc_max_sym - s_lat_acc_sym});

        /* Vcmd <= Vmax */
//        sym_t Vmax = sym_t::sqrt(lat_acc_max_sym / curvature);
//        ineq_constr = sym_t::vertcat({ineq_constr, Vcmd - Vmax});                       // Gives INVALID_NUMBER

        /* Cnd^2 <= Vmax^2 */
//        sym_t Vmax2 = lat_acc_max_sym / curvature;
//        ineq_constr = sym_t::vertcat({ineq_constr, sym_t::sq(Vcmd) - Vmax2}); // <= 0   // Gives PROBLEM_INFEASIBLE
//        std::cout << "ineq_constr: " << ineq_constr << "\n";

        /* Upper bound on throttle according to continuous max throttle profile */
        // ineq_constr = sym_t::vertcat({
        //     ineq_constr,
        //     u(1) <= throttle_max_spline(gamma)[0]
        // });

#if USE_TRACKBOUNDS
        if (0 < i) // && i <= 4)
        {
            /* Contouring constraints (local linear approximations) */
            sym_t pos_ref = path_spline.spline_func(gamma)[0];

            /* Contouring constraints (local linear approximations) */
            sym_t spline_angle = path_spline.spline_func(gamma)[3];
//            sym_t T_lk = get_Tz(spline_angle);
//            sym_t pos_diff = pos - pos_ref;
//            sym_t t_pos_diff = sym_t::mtimes(T_lk, pos_diff);
//            sym_t t_pos_diff_y = t_pos_diff(1);
            sym_t t_pos_diff_y = sym_t::mtimes(sym_t::horzcat({-sin(spline_angle), cos(spline_angle)}), (pos - pos_ref));
//            sym_t grad = path_spline.spline_func(gamma)[1];
//            sym_t cross_grad = sym_t::horzcat({-grad(1), grad(0)});
//            sym_t t_pos_diff_y = sym_t::mtimes(cross_grad / sym_t::norm_2(cross_grad), (pos_ref - pos));

            // Constant lateral offset (if spline is centerline)
            std::vector<sym_t> offsLR_ =  {0.5 * track->get_info().width, -0.5 * track->get_info().width};

            // Varying lateral offset (if spline is raceline)
            //            std::vector<sym_t> offsLR_ = offsLR_spline.spline_func(gamma);

            const double car_width = tuningParameters.car_width;
            ineq_constr = sym_t::vertcat({ineq_constr,
                                          t_pos_diff_y - (offsLR_[0] - 0.5 * car_width) , //- s_contour_sym,
                                          -(-offsLR_[1] - 0.5 * car_width) - t_pos_diff_y , //- s_contour_sym
                                         });
        }
#endif
        return ineq_constr;
    }

protected:
    spline_following::casadi_ocp::CasadiSpline path_spline;
    spline_following::casadi_ocp::CasadiSpline offsLR_spline;

    sym_t x0_sym;
    sym_t gamma0_sym;

    sym_t w_path_progression_sym;
    sym_t w_path_tracking_sym;
    sym_t R_diag_sym;
    sym_t mayer_factor_sym;

//    sym_t lat_acc_max_sym;
//    sym_t s_lat_acc_sym;

//    sym_t s_contour_sym;

    // sym_t throttle_max_profile_sym;

    void shift_gamma()
    {
        if (solution_avail)
        {
            DM path_param_warm = X_opt(state_idx.path_param, casadi::Slice());
            if (shift_warm_path_param(state.path_param, path_param_warm))
            {
                opti->set_initial(X_sym(state_idx.path_param, casadi::Slice()), path_param_warm);
            }
        }
    }
    void shift_heading()
    {
        if (solution_avail)
        {
            DM heading_warm = X_opt(state_idx.turn_angle, casadi::Slice());
            if (mpc_control_racing::casadi_ocp::shift_warm_heading(state.turn_angle, heading_warm))
            {
                opti->set_initial(X_sym(state_idx.turn_angle, casadi::Slice()), heading_warm);
            }
        }
    }

private:
    utils_racing::PathLocator track_locator;
};

}
}
}

#endif //SRC_SPLINEFOLLOWINGOPTIOCP
