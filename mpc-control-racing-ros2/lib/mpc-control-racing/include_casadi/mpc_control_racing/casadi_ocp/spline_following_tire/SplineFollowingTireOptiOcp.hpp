#ifndef SRC_SPLINEFOLLOWINGTIREOPTIOCP
#define SRC_SPLINEFOLLOWINGTIREOPTIOCP

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/casadi_model/StackedSystem.hpp"
#include "mpc_control_racing/casadi_model/dynamic_bicycle/DynamicBicycle.hpp"
//#include "mpc_control_racing/casadi_model/dynamic_bicycle/DynamicBicycleWithActuators.hpp"
#include "mpc_control_racing/casadi_model/AcceleratedPathparam.hpp"
#include "mpc_control_racing/casadi_model/ExtraInput.hpp"

#include "mpc_control_racing/casadi_ocp/spline_following/CasadiSpline.hpp"

#include "mpc_control_racing/shift_warm_start_heading_casadi.hpp"
#include "utils_racing/PathUtils.hpp"

#include "mpc_control_racing/problem/spline_following/SplineFollowingTranscriptionSettings.hpp"

#define USE_U_CMD false

#define USE_TRACKBOUNDS true
#define USE_SINGLE_SLACK false

#define USE_TRACKBOUNDS_COST false

namespace mpc_control_racing {
namespace spline_following {
namespace casadi_ocp {

using namespace casadi_model;

using DynamicBicycleWithTrackingMX = StackedSystem<DynamicBicycle<casadi::MX>, AcceleratedPathparam<casadi::MX>>;

class SplineFollowingTireOptiOcp : public mpc_control_common::ocp_casadi::OptiOcp,
                                   public mpc_control_racing::RacingOptimizerInterface
{
public:
    SplineFollowingTireOptiOcp()
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
        state.turn_angle = 0;
        state.vel = {0.2, 0.0};
        state.turn_angle = 0.0;
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
    }
    bool init() override
    {
#if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        StackedSystem<DynamicBicycleWithTrackingMX, ExtraInput<casadi::MX>> model;
#else
        DynamicBicycleWithTrackingMX model;
#endif

//        DM x0 = DM::vertcat({0.510, -0.126, -0.207,  0.501, -0.001, -0.001,  0.048,  0.047});
//        DM u = DM::vertcat({0.4, 0.2, 0.05});
//        DM p = DM::vertcat({});
//        DM d = DM::vertcat({});
//        std::cout << "Model evaluation test " << model.dynamics(casadi::DMVector({x0, u, p, d}))[0].nonzeros() << "\n";
        // Model is identical

//        std::cout << "Spline evaluation test " << fspline(DM(0.147))[0] << " "
//                  << fspline(DM(0.151))[0] << " "
//                  << fspline(DM(0.153))[0] << "\n";
        // Spline is identical

        init_base(model.dynamics, horizon);

        /* Symbolic problem parameters */
        x0_sym = opti->parameter(nx); // Symbolic initial state
        gamma0_sym = opti->parameter(1);

        w_path_tracking_sym = opti->parameter(); // Symbolic path tracking weight
        w_path_progression_sym = opti->parameter(); // Symbolic path progression weight
        W_diag_v_body_sym = opti->parameter(2); // Symbolic body vel weight
        w_turn_rate_sym = opti->parameter();

        R_diag_sym = opti->parameter(3); // Symbolic input weight diagonal
        mayer_factor_sym = opti->parameter(1); // Symbolic mayer factor

#if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        {
            s_contour_sym = opti->variable(1);
        }
#endif
//        lat_acc_max_sym = opti->parameter(1); // Maximum lateral acceleration

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

        tuningParameters.w_slack_contour = yamlNode["w_slack_contour"].as<double>();

        const std::vector<double> R_du_diag = yamlNode["R_du_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_du_diag.size(); i++) { tuningParameters.R_du_diag(i) = R_du_diag[i]; }

        const std::vector<double> R_prev_diag_ = yamlNode["R_prev_diag"].as<std::vector<double>>();
        for (unsigned i = 0; i < R_prev_diag_.size(); i++) { R_prev_diag(i) = R_prev_diag_[i]; }

        const std::vector<double> W_diag_v_body = yamlNode["W_diag_v_body"].as<std::vector<double>>();
        tuningParameters.W_diag_v_body << W_diag_v_body[0], W_diag_v_body[1];

        tuningParameters.w_turn_rate = yamlNode["w_turn_rate"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Spline Following Tire OCP -- \n"
                  << "w_path_tracking: " << tuningParameters.w_path_tracking << "\n"
                  << "w_path_progression: " << tuningParameters.w_path_progression << "\n"
                  << "w_slack_contour: " << tuningParameters.w_slack_contour << "\n"
                  << "R_du_diag: " << tuningParameters.R_du_diag.transpose() << "\n"
                  << "R_prev_diag: " << R_prev_diag << "\n"
                  << "W_diag_v_body: " << tuningParameters.W_diag_v_body.transpose() << "\n"
                  << "w_turn_rate: " << tuningParameters.w_turn_rate << "\n"
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

        Eigen::Matrix2d T_body_world;
        T_body_world << std::cos(state.turn_angle), std::sin(state.turn_angle),
                -std::sin(state.turn_angle), std::cos(state.turn_angle);

        Eigen::Vector2d vel_body = T_body_world * state.vel;
        // std::cout << "vel_body: " << vel_body << std::endl;
        vel_body(0) = std::max(0.501, vel_body(0)); // Project vx onto speed range (lower bound)
        vel_body(1) = std::max(-0.049, std::min(vel_body(1), 0.049)); // Project vy

        // Estimate path param velocity
        state.path_param_vel = vel_body(0) / track->get_info().length;

//        DM grad = path_spline.spline_func(DM(state.path_param))[1];
//        grad = grad / DM::norm_2(grad);
//        DM orth = DM::vertcat({-grad(1), grad(0)});
//        DM T_lk = DM::horzcat({grad, orth}).T();
//        DM l_vel = DM::mtimes(T_lk, eigen_casadi_conversion::toDM(state.vel));
//
//        state.path_param_vel = l_vel(0).nonzeros()[0] / track->get_info().length;

        // Assemble state from available measurements in state struct
        if (USE_U_CMD)
        {
            if (solve_succeeded)
            {
                // Use previously planned control as initial control for control evolution
                state.steering = get_control_trajectory()(0, 1);
                state.throttle = get_control_trajectory()(1, 1);
            }
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate,
                    state.steering, state.throttle, state.path_param, state.path_param_vel;
        }
        else
        {
            x_comp << state.pos, state.turn_angle, vel_body, state.turn_rate,
                    state.path_param, state.path_param_vel;
        }
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
        opti->set_value(w_path_tracking_sym, tuningParameters.w_path_tracking);
        opti->set_value(w_path_progression_sym, tuningParameters.w_path_progression);

        opti->set_value(R_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_diag));
//        opti->set_value(R_du_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_du_diag.topRows(2)));

        opti->set_value(W_diag_v_body_sym, eigen_casadi_conversion::toDM(tuningParameters.W_diag_v_body));
        opti->set_value(w_turn_rate_sym, tuningParameters.w_turn_rate);

        opti->set_value(mayer_factor_sym, tuningParameters.mayer_factor);
//        opti->set_value(lat_acc_max_sym, tuningParameters.lat_acc_max);

        tuningParameters.needs_update = false;
    }

    /* Problem definition */
    void setup_discretized_trajectory_constraints() override
    {
        Slice all;
        {
            // Box constraints on input
//            set_ubu(DM::vertcat({1, tuningParameters.throttle_max, 1}));
//            set_lbu(DM::vertcat({-1, tuningParameters.throttle_min, -1}));
            // To update throttle_max based on manual cap, need to use throttle_max_sym and update throttle_max_sym
            // in update_config_params()
//            opti->subject_to(-1 <= U_sym(0, all) <= 1);
//            opti->subject_to(throttle_min_sym <= U_sym(1, all) <= throttle_max_sym);
//            opti->subject_to(-1 <= U_sym(2, all) <= 1); // Steering, throttle, path parameter acceleration

            // Move blocking on first couple inputs
//            const unsigned join_n_inputs = 1; // 1 means no move blocking
//            for (unsigned k = 0; k < join_n_inputs - 1; k++)
//            {
//                std::cout << "Constraint: input " << k << " == input " << k + 1 << "\n";
//                sym_t du = (U_sym(all, k + 1) - U_sym(all, k));
////                opti->subject_to(U_sym(all, k) == U_sym(all, k + 1));
//                opti->minimize(opti->f() + 1000 * sym_t::mtimes(du.T(), du));
//            }

            /* Differential constraint on input */
//            for (unsigned iu = 0; iu < U_sym.size1(); iu++)
//            {
//                if (tuningParameters.R_du_diag(iu) > 0)
//                {
//                    std::cout << "Cost on discrete input differences for input " << iu << "\n";
//                    for (unsigned k = 1; k < U_sym.size2(); k++)
//                    {
//                        sym_t du = (U_sym(all, k) - U_sym(all, k - 1));
//                        opti->minimize(opti->f() + tuningParameters.R_du_diag(iu) * sym_t::sq(du(iu)));
//                    }
//                }
//            }
        }

        // Box constraints on state
//        opti->subject_to(0.2 <= X_sym(state_idx.vel_x, all)); // 0.1 < vx
////        opti->subject_to(-0.05 <= X_sym(state_idx.vel_y, all) <= 0.05); // vy
//
//        opti->subject_to(0 <= X_sym(state_idx.path_param, all) <= 2); // Path parameter can only be in (positive) path range
//        opti->subject_to(0 <= X_sym(state_idx.path_param_vel, all) <= 1); // Path velocity positive

        // Nonlinear constraint on state/input/parameter
//        for (unsigned i = 0; i < U_sym.size2(); i++)
//        {
//            /* Constraint on lateral acceleration */
//            sym_t steering = U_sym(0, i);
//            sym_t V_cmd = U_sym(1, i);
//            opti->subject_to(-lat_acc_max_sym < sym_t::sq(V_cmd) * steering < lat_acc_max_sym);
//        }

        // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega, path, path vel)
//        casadi::DM x0_epsi_dynbcl = DM::vertcat({0.005, 0.005, 0.005, 0.01, 0.005, 0.005});
//        casadi::DM x0_epsi_path = {0.05, 0.01};
//
//        casadi::MX epsi;
//        if (USE_U_CMD) { epsi = DM::vertcat({x0_epsi_dynbcl, 0.999, 0.499, x0_epsi_path}); }
//        else { epsi = DM::vertcat({x0_epsi_dynbcl, x0_epsi_path}); }
//        opti->subject_to(x0_sym - epsi <= X_sym(all, 0) <= x0_sym + epsi);
//        opti->subject_to(x0_sym <= X_sym(all, 0) <= x0_sym);

        // Boundary conditions: Final state

        // One slack variable for all contouring constraints
//#if (USE_TRACKBOUNDS)
//        {
//            opti->subject_to({0 <= s_contour_sym});
//        }
//#endif
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
        const sym_t pos = x(Slice(state_idx.pos_x, state_idx.pos_y + 1));
        const sym_t gamma = x(state_idx.path_param);
        (void) p;
        (void) d;

        sym_t non_control_cost = 0;

        const sym_t pos_ref = path_spline.spline_func(gamma)[0];
//        const sym_t pos_ref = DM::vertcat({5, 3});
        const sym_t pos_err = pos_ref - pos;
        non_control_cost += w_path_tracking_sym * sym_t::dot(pos_err, pos_err);

        const sym_t body_vel = x(Slice(state_idx.vel_x, state_idx.vel_y + 1));
        non_control_cost += sym_t::dot(body_vel, sym_t::mtimes(sym_t::diag(W_diag_v_body_sym), body_vel));

        const sym_t turn_rate = x(state_idx.turn_rate);
        non_control_cost += w_turn_rate_sym * sym_t::sq(turn_rate);

        non_control_cost += -w_path_progression_sym * (gamma - gamma0_sym);

        /* Using path param velocity cost results in poor numerical solvability! */
        //        const T gamma_vel = x(state_idx.path_param_vel);
        //        non_control_cost += -tuningParameters.w_path_progression * gamma_vel;

        if (USE_TRACKBOUNDS_COST)
        {
            /* Contouring constraints (local linear approximations) */
            sym_t spline_angle = path_spline.spline_func(gamma)[3];
            sym_t T_lk = get_Tz(spline_angle);
            sym_t pos_diff = pos - pos_ref;
            sym_t t_pos_diff = sym_t::mtimes(T_lk, pos_diff);

            // Constant lateral offset (if spline is centerline)
             sym_t offsLR_ = 0.5 * track->get_info().width * sym_t::horzcat({1, -1});

            // Varying lateral offset (if spline is raceline)
//            std::vector<sym_t> offsLR_ = offsLR_spline.spline_func(gamma);

            const double car_width = tuningParameters.car_width;
            const double margin = tuningParameters.lat_acc_max;
            const sym_t wallL = exp(100.0 * (t_pos_diff(1) - (offsLR_(0) - 0.5 * car_width - margin)));
            const sym_t wallR = exp(100.0 * (-(-offsLR_(1) - 0.5 * car_width - margin) - t_pos_diff(1)));
            non_control_cost += wallL + wallR;
        }
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

        sym_t lagrange_cost = 0;
        lagrange_cost +=get_non_control_cost(x, p, d) + get_control_cost(u(Slice(0, 3)), p, d); //+ costV;

        /* Track maximum possible speed */
//        sym_t gamma = x(3);
//        sym_t curvature = fspline(gamma)[1];
//        sym_t Vmax2 = lat_acc_max_sym / curvature;
//
//        sym_t Vcmd = u(1);
//        sym_t Verr = Vmax2 - Vcmd;
//        sym_t costV = sym_t::sq(Verr);

#if (USE_TRACKBOUNDS && !USE_SINGLE_SLACK)
        sym_t s_contour = u(3);
        lagrange_cost += tuningParameters.w_slack_contour * (s_contour * s_contour + s_contour);
#endif
        return lagrange_cost;
    }

    sym_t mayer_cost_impl(const sym_t &xf, const sym_t &p, const sym_t &d, const sym_t &t) override
    {
        (void) p;
        (void) d;
        (void) t;

        sym_t mayer_cost(0);

        mayer_cost += mayer_factor_sym * get_non_control_cost(xf, p, d);

        /* Using path param progression again in mayer term results in poor yaw stabilization at the beginning of the horizon! */
//        - w_path_progression_sym * (xf(6) - gamma0_sym);

#if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        sym_t s_contour = s_contour_sym;
        mayer_cost += 1e3 * (s_contour * s_contour + s_contour);
#endif
        return mayer_cost;
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
        const sym_t pos = x(Slice(state_idx.pos_x, state_idx.pos_y + 1));
        const sym_t gamma = x(state_idx.path_param);
        (void) p;
        (void) d;
        (void) t;
        unsigned i = static_cast<unsigned> (std::round(DM(t).nonzeros()[0] / (tf / MS_N_SEGS)));

        sym_t ineq_constr = {};

        if (i == 0)
        {
            // Boundary conditions: Initial state (X, Y, theta, vx, vy, omega, path, path vel)
            casadi::DM x0_epsi_dynbcl = DM::vertcat({0.005, 0.005, 0.005, 0.01, 0.005, 0.005});
            casadi::DM x0_epsi_path = {0.05, 0.01};

            casadi::MX epsi = casadi::MX::zeros(x.size1(), 1);
            if (USE_U_CMD) { epsi = DM::vertcat({x0_epsi_dynbcl, 0.999, 0.499, x0_epsi_path}); }
            else { epsi = DM::vertcat({x0_epsi_dynbcl, x0_epsi_path}); }

            ineq_constr = sym_t::vertcat({ineq_constr,
                                          (x0_sym - epsi) - x, x - (x0_sym + epsi),
                                         });
        }
        // State constraints
        ineq_constr = sym_t::vertcat({ineq_constr,
                                    0.2 - x(state_idx.vel_x),
                                    0 - x(state_idx.path_param), x(state_idx.path_param) - 2,
                                    0 - x(state_idx.path_param_vel), x(state_idx.path_param_vel) - 1,
                                    });

        // Input constraints
        DM ubu = DM::vertcat({1, tuningParameters.throttle_max, 1}); // Steering, throttle, path parameter acceleration
        DM lbu = DM::vertcat({-1, tuningParameters.throttle_min, -1});
        ineq_constr = sym_t::vertcat({ineq_constr,
                                      lbu - u(Slice(0, 3)), u(Slice(0, 3)) - ubu});

#if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
        ineq_constr = sym_t::vertcat({ineq_constr,
                                      0 - s_contour_sym});
#else
        ineq_constr = sym_t::vertcat({ineq_constr,
                                      0 - u(3)});
#endif

//        sym_t gamma = x(6);
//        sym_t curvature = fspline(gamma)[1];
//        sym_t Vcmd = 3.0 * u(1);

//        ineq_constr = sym_t::vertcat({ineq_constr,
//                                      Vcmd * Vcmd * abs(curvature) - lat_acc_max_sym});

        /* Vcmd <= Vmax */
//        sym_t Vmax = sym_t::sqrt(lat_acc_max_sym / curvature);
//        ineq_constr = sym_t::vertcat({ineq_constr, Vcmd - Vmax});                       // Gives INVALID_NUMBER

        /* Cnd^2 <= Vmax^2 */
//        sym_t Vmax2 = lat_acc_max_sym / curvature;
//        ineq_constr = sym_t::vertcat({ineq_constr, sym_t::sq(Vcmd) - Vmax2}); // <= 0   // Gives PROBLEM_INFEASIBLE
//        std::cout << "ineq_constr: " << ineq_constr << "\n";

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
//            std::vector<sym_t> offsLR_ =  {0.5 * track->get_info().width, -0.5 * track->get_info().width};

            // Varying lateral offset (if spline is raceline)
            std::vector<sym_t> offsLR_ = offsLR_spline.spline_func(gamma);

            const double car_width = tuningParameters.car_width;
#if USE_SINGLE_SLACK
            sym_t s_contour = s_contour_sym;
#else
            sym_t s_contour = u(3);
#endif
            ineq_constr = sym_t::vertcat({ineq_constr,
                                          t_pos_diff_y - (offsLR_[0] - 0.5 * car_width) - s_contour,
                                          -(-offsLR_[1] - 0.5 * car_width) - t_pos_diff_y - s_contour
                                         });
        }
#endif
        return ineq_constr;
    }

protected:
    DM R_prev_diag{1e-4, 1e-4};

    sym_t x0_sym;
    sym_t gamma0_sym;

    sym_t w_path_tracking_sym;
    sym_t w_path_progression_sym;
    sym_t W_diag_v_body_sym;
    sym_t w_turn_rate_sym;

    sym_t R_diag_sym;
    // sym_t R_prev_diag;

    sym_t mayer_factor_sym;

//    sym_t lat_acc_max_sym;

#if (USE_TRACKBOUNDS && USE_SINGLE_SLACK)
    sym_t s_contour_sym;
#endif

    void shift_gamma()
    {
        if (solution_avail)
        {
            DM path_param_warm = X_opt(state_idx.path_param, Slice());
            if (shift_warm_path_param(state.path_param, path_param_warm))
            {
                opti->set_initial(X_sym(state_idx.path_param, Slice()), path_param_warm);
            }
        }
    }
    void shift_heading()
    {
        if (solution_avail)
        {
            DM heading_warm = X_opt(state_idx.turn_angle, Slice());
            if (mpc_control_racing::casadi_ocp::shift_warm_heading(state.turn_angle, heading_warm))
            {
                opti->set_initial(X_sym(state_idx.turn_angle, Slice()), heading_warm);
            }
        }
    }

    DM generate_X_guess(const DM &x0, const double dt, const unsigned N_points) const
    {
        DM X_guess = DM::repmat(x0, 1, N_points);
        const int px = state_idx.pos_x;
        const int py = state_idx.pos_y;
        const int thet = state_idx.turn_angle;
        const int vx = state_idx.vel_x;
        const int vy = state_idx.vel_y;
        const int w = state_idx.turn_rate;

        for (unsigned i = 0; i < N_points - 1; i++)
        {
            DM x = X_guess(Slice(), i);
            DM x_next = x;

            // Integrate heading and position
            x_next(thet) += x(w) * dt;
            x_next(px) += (x(vx) * cos(x_next(thet)) - x(vy) * sin(x_next(thet))) * dt; // vx * cos(theta) - vy * sin(theta)
            x_next(py) += (x(vx) * sin(x_next(thet)) + x(vy) * cos(x_next(thet))) * dt; // vx * sin(theta) + vy * cos(theta)

            x_next(state_idx.path_param) += x(state_idx.path_param_vel) * dt; // Integrate path param velocity

            X_guess(Slice(), i + 1) = x_next;
        }
        return X_guess;
    }

private:
    using Slice = casadi::Slice;
    //    casadi::Function fspline = LA_track_spline();
    spline_following::casadi_ocp::CasadiSpline path_spline;
    spline_following::casadi_ocp::CasadiSpline offsLR_spline;

    utils_racing::PathLocator track_locator;
};

}
}
}

#endif //SRC_SPLINEFOLLOWINGTIREOPTIOCP
