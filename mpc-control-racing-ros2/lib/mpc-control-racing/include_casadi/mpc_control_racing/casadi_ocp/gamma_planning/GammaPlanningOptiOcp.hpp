#ifndef SRC_GAMMAPLANNINGOPTIOCP
#define SRC_GAMMAPLANNINGOPTIOCP

#include "casadi/casadi.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

#include "mpc_control_common/casadi_ocp/OptiOcp.hpp"
#include "mpc_control_racing/RacingOptimizerInterface.hpp"

#include "mpc_control_racing/problem/gamma_planning/GammaPlanningTranscriptionSettings.hpp"
#include "mpc_control_racing/casadi_ocp/spline_following/CasadiSpline.hpp"

namespace mpc_control_racing {
namespace gamma_planning {
namespace casadi_ocp {

static casadi::Function get_gamma_model()
{
    using namespace casadi;

    MX gamma = MX::sym("gamma");
    MX gamma_vel = MX::sym("gamma_vel");
    MX gamma_acc = MX::sym("gamma_acc");

    MX state = MX::vertcat({gamma, gamma_vel});
    MX input = gamma_acc;
    MX state_dot = MX::vertcat({gamma_vel, gamma_acc});

    MX p = MX::sym("p");
    MX d = MX::sym("d");

    Function dynamics_fun = Function("path_dynamics", {state, input, p, d}, {state_dot});
    return dynamics_fun;
}

class GammaPlanningOptiOcp : public mpc_control_common::ocp_casadi::OptiOcp,
                             public mpc_control_racing::RacingOptimizerInterface
{
public:
    GammaPlanningOptiOcp()
    {
        /* Define state */
        state_idx.path_param = 0;
        state_idx.path_param_vel = 1;

        /* Initialize state */
        state.path_param = 0;
        state.path_param_vel = 1;
        x_comp.resize(2);
        x_comp.setZero();

        /* Config parameters */
        configParameters.path_params.resize(MS_N_SEGS + 1);
        configParameters.raceline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.grad_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_refs.resize(MS_N_SEGS + 1, 2);
        configParameters.centerline_grad_refs.resize(MS_N_SEGS + 1, 2);

        configParameters.path_params.setZero();
        configParameters.path_velocities.setZero();
        configParameters.raceline_refs.setZero();
        configParameters.grad_refs.setZero();
        configParameters.centerline_refs.setZero();
        configParameters.centerline_grad_refs.setZero();

        /* Default tuning parameters */
        tuningParameters.w_path_progression = 1;
        tuningParameters.R_diag.topRows(2) << 1, 1;
        tuningParameters.mayer_factor = 1;
    }
    // spline_following::casadi_ocp::CasadiSpline spline;
    std::shared_ptr<spline_following::casadi_ocp::CasadiSpline_Path> spline = std::make_shared<spline_following::casadi_ocp::CasadiSpline_Path>();

    double m_path_length{10};
    const double p_speed_gain = 6;

    bool init() override
    {
        const casadi::Function cont_dyn = get_gamma_model();
        init_base(cont_dyn, horizon);

        /* Symbolic OptiStack parameters */
        x0_sym = opti->parameter(nx); // Symbolic initial state

        w_path_progression_sym = opti->parameter(1);

        R_diag_sym = opti->parameter(nu); // Symbolic input weight diagonal
        mayer_factor_sym = opti->parameter(1); // Symbolic mayer factor

        lat_acc_max_sym = opti->parameter(1);

        /* Load path length */
        // m_path_length = track->get_info().length;
        m_path_length = spline->get_path_length();

        return true;
    }


    /* Updaters for state and parameters */
    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        // Load common parameters
        RacingOptimizerInterface::load_params_from_yaml(yaml_filepath);

        // Load OCP specific parameters
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        tuningParameters.lon_acc_max = yamlNode["lon_acc_max"].as<double>();
        tuningParameters.w_path_progression = yamlNode["w_path_progression"].as<double>();

        configParameters.needs_update = true;
        tuningParameters.needs_update = true;

        std::cout << " -- Gamma Planning -- \n"
                  << "lon_acc_max: " << tuningParameters.lon_acc_max << "\n"
                  << "w_path_progression: " << tuningParameters.w_path_progression << "\n"
                  << "\n";
    }
    void load_spline_from_yaml(const std::string &yaml_filepath)
    {
        spline->load_from_yaml(yaml_filepath, "pos", spline_following::casadi_ocp::CasadiSpline::Pos);
    }

    Eigen::VectorXd compose_state() override
    {
        state.path_param = track_locator.grid_search(spline, state.pos, state.path_param);

        // double tangent_angle = track->eval_tangent_angle(state.path_param);
        const casadi::DM grad = spline->spline_func(casadi::DM(state.path_param))[1];
        const double tangent_angle = casadi::DM::atan2(grad(1), grad(0)).nonzeros()[0];
        Eigen::Matrix2d T_lk;
        T_lk << std::cos(tangent_angle), std::sin(tangent_angle),
                -std::sin(tangent_angle), std::cos(tangent_angle);
        const Eigen::VectorXd l_vel = T_lk * state.vel;
        state.path_param_vel = l_vel(0) / m_path_length;

        const double Vmin = tuningParameters.throttle_min * p_speed_gain;
        state.path_param_vel = std::max(Vmin / m_path_length, state.path_param_vel);

        // Assemble state from available measurements in state struct
        x_comp << state.path_param, state.path_param_vel;
        return x_comp;
    }
    void update_config_params() override
    {
        configParameters.needs_update = false;
    }
    void update_tuning_params() override
    {
        opti->set_value(R_diag_sym, eigen_casadi_conversion::toDM(tuningParameters.R_diag.bottomRows(1)));
        opti->set_value(mayer_factor_sym, tuningParameters.mayer_factor);

        opti->set_value(w_path_progression_sym, tuningParameters.w_path_progression);

        opti->set_value(lat_acc_max_sym, tuningParameters.lat_acc_max);

        tuningParameters.needs_update = false;
    }

    /* Problem definition */
    void setup_discretized_trajectory_constraints() override
    {
        // Box constraints on input
        set_ubu(tuningParameters.lon_acc_max / m_path_length); // Path parameter acceleration
        set_lbu(-tuningParameters.lon_acc_max / m_path_length);

        set_ubx(casadi::DM::vertcat({DM::inf(), tuningParameters.throttle_max * p_speed_gain / m_path_length})); // Vel
        set_lbx(casadi::DM::vertcat({0, 0.01})); //tuningParameters.throttle_min * p_speed_gain / m_path_length}));

        // Boundary conditions: Initial state
        casadi::Slice all;
        casadi::MX epsi = casadi::DM::vertcat({0, 0});
        opti->subject_to(x0_sym - epsi <= X_sym(all, 0) <= x0_sym + epsi);

        // Boundary conditions: Final state
    }

    sym_t get_non_control_cost(const sym_t &x, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        (void) p;
        (void) d;
        (void) t;
        sym_t gamma = x(0);
        sym_t gamma_vel = x(1);
        sym_t non_control_cost = 0;
        non_control_cost += -w_path_progression_sym * gamma;
//        non_control_cost += -w_path_progression_sym * gamma_vel;
        return non_control_cost;
    }
    sym_t get_control_cost(const sym_t &u, const sym_t &p, const sym_t &d, const sym_t &t)
    {
        (void) p;
        (void) d;
        (void) t;
        sym_t control_cost = sym_t::mtimes(u.T(), sym_t::mtimes(sym_t::diag(R_diag_sym), u));
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
        (void) p;
        (void) d;
        return mayer_factor_sym * get_non_control_cost(xf, p, d, tf);
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
        (void) x;
        (void) u;
        (void) p;
        (void) d;
        (void) t;

        sym_t ineq_constr = {};

        sym_t gamma = x(0);
        sym_t gamma_vel = x(1);

        sym_t curvature = spline->spline_func(gamma)[4];
        sym_t V = gamma_vel * m_path_length;

        ineq_constr = sym_t::vertcat({ineq_constr,
                                      sym_t::sq(V) * curvature - lat_acc_max_sym, // <= 0
                                      -lat_acc_max_sym - sym_t::sq(V) * curvature
                                     });
        return ineq_constr;
    }

protected:
    utils_racing::PathLocator track_locator;

    sym_t x0_sym;

    sym_t w_path_progression_sym;

    sym_t R_diag_sym;
    sym_t mayer_factor_sym;

    sym_t lat_acc_max_sym;

    void shift_gamma()
    {
        if (solution_avail)
        {
            DM path_param_warm = X_opt(0, casadi::Slice());
            if (spline_following::casadi_ocp::shift_warm_path_param(state.path_param, path_param_warm))
            {
                opti->set_initial(X_sym(0, casadi::Slice()), path_param_warm);
            }
        }
    }
};

}
}
}

#endif //SRC_GAMMAPLANNINGOPTIOCP
