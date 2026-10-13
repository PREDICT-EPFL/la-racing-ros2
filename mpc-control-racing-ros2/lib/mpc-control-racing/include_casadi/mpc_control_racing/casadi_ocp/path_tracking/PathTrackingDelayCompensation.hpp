#ifndef SRC_PATHTRACKINGDELAYCOMPENSATION_HPP
#define SRC_PATHTRACKINGDELAYCOMPENSATION_HPP

#include "mpc_control_racing/casadi_ocp/path_tracking/PathTrackingOptiOcp.hpp"

#include "mpc_control_common/DelayCompensation.hpp"
#include "mpc_control_common/casadi_transcription/Integration.hpp"

namespace mpc_control_racing{
namespace path_tracking{
namespace casadi_ocp{
class KinIntegrator_DelayCompensator : public DelayCompensator
{
public:
    Eigen::VectorXd cont_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
//        const Eigen::Vector2d pos = x.middleRows(0, 2);
//        const double turn_angle = x(2);
        const Eigen::Vector2d vel = x.middleRows(3, 2);
        const double turn_rate = x(5);

        Eigen::VectorXd x_dot = x;
        x_dot << vel, turn_rate, Eigen::Vector2d::Zero(), 0; // Assume zero acceleration
        return x_dot;
    }
    Eigen::VectorXd compose_delayed_state(const RacingOcpParams::RacingState &state, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x_comp(6);
        x_comp << x0(state_index.pos_x), x0(state_index.pos_y), x0(state_index.turn_angle), state.vel(0), state.vel(1), state.turn_rate;
        return x_comp;
    }
    Eigen::VectorXd get_ocp_state(const Eigen::VectorXd &x_pred, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x_ocp(3);

        x_ocp(state_index.pos_x) = x_pred(0);
        x_ocp(state_index.pos_y) = x_pred(1);
        x_ocp(state_index.turn_angle) = x_pred(2);

        return x_ocp;
    }
    KinIntegrator_DelayCompensator() : DelayCompensator(DynamicsMode::CONTINUOUS, 0.005)
    {
    }
};

class KinBicycle_DelayCompensator : public DelayCompensator
{
public:
    casadi_model::SimpleBicycleMX model;
    casadi::Function model_integrator;

    Eigen::VectorXd cont_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
        using namespace casadi;
        DM x_dot = model.dynamics(DMVector({eigen_casadi_conversion::toDM(x),
                                            DM({u(0), u(1) / model.speed_gain}),
                                            DM(), DM()}))[0];
        return eigen_casadi_conversion::toEigenX(x_dot);
    }
    Eigen::VectorXd discr_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
        using namespace casadi;
        DM x_next = model_integrator(DMVector({eigen_casadi_conversion::toDM(x),
                                               DM({u(0), u(1) / model.speed_gain}),
                                               DM(), DM()}))[0];
        return eigen_casadi_conversion::toEigenX(x_next);
    }
    Eigen::VectorXd compose_delayed_state(const RacingOcpParams::RacingState &state, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x_comp(3);

        x_comp(state_index.pos_x) = x0(state_index.pos_x);
        x_comp(state_index.pos_y) = x0(state_index.pos_y);
        x_comp(state_index.turn_angle) = x0(state_index.turn_angle);

        return x_comp;
    }
    Eigen::VectorXd get_ocp_state(const Eigen::VectorXd &x_pred, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x_ocp(3);

        x_ocp(state_index.pos_x) = x_pred(state_index.pos_x);
        x_ocp(state_index.pos_y) = x_pred(state_index.pos_y);
        x_ocp(state_index.turn_angle) = x_pred(state_index.turn_angle);

        return x_ocp;
    }
    explicit KinBicycle_DelayCompensator(const DynamicsMode &dynamicsMode = DynamicsMode::CONTINUOUS) : DelayCompensator(dynamicsMode, 0.005)
    {
        // Initialize integration stepper for the discrete case
        using namespace casadi;
        MX x = MX::sym("x", 3);
        MX u = MX::sym("u", 2);
        MX p = MX::sym("p", 0);
        MX d = MX::sym("d", 0);
        MX xp = mpc_control_common::transcription_casadi::integration::RK4_int(model.dynamics,
                                                                               x, u, p, d, MX(m_dt));
        model_integrator = Function("model_int", {x, u, p, d}, {xp});
    }
};

std::shared_ptr<DelayCompensator> build_delay_compensator(const int delay_compensation_mode)
{
    std::shared_ptr<DelayCompensator> delay_compensator;
    switch(delay_compensation_mode)
    {
        case 1:
            delay_compensator = std::make_shared<KinIntegrator_DelayCompensator>();
            break;
        case 2:
            delay_compensator = std::make_shared<KinBicycle_DelayCompensator>(DelayCompensator::DynamicsMode::CONTINUOUS);
            break;
        case 3:
            delay_compensator = std::make_shared<KinBicycle_DelayCompensator>(DelayCompensator::DynamicsMode::DISCRETE);
            break;
        default:
            delay_compensator = nullptr;
            break;
    }

    return delay_compensator;
}
}
}
}

#endif //SRC_PATHTRACKINGDELAYCOMPENSATION_HPP
