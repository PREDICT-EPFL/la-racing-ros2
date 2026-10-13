#ifndef SRC_SPLINEFOLLOWINGTIREDELAYCOMPENSATION_HPP
#define SRC_SPLINEFOLLOWINGTIREDELAYCOMPENSATION_HPP

#include "mpc_control_racing/problem/spline_following_tire/laopt_ocp/SplineFollowingTireLaoptOcp.hpp"

#include "mpc_control_common/DelayCompensation.hpp"
#include <boost/numeric/odeint.hpp>

namespace mpc_control_racing{
namespace spline_following{
namespace laopt_ocp{

class KinIntegrator_DelayCompensator : public DelayCompensator
{
public:
    Eigen::VectorXd cont_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
        (void) u;
//        const Eigen::Vector2d pos = x.middleRows(0, 2);
//        const double turn_angle = x(2);
        const Eigen::Vector2d vel = x.segment<2>(3);
        const double turn_rate = x(5);

        Eigen::VectorXd x_dot = x;
        x_dot << vel, turn_rate, Eigen::Vector2d::Zero(), 0; // Assume zero acceleration
        return x_dot;
    }
    Eigen::VectorXd compose_delayed_state(const RacingOcpParams::RacingState &state, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        Eigen::VectorXd x_comp(6);
        x_comp << x0(state_index.pos_x), x0(state_index.pos_y), x0(state_index.turn_angle), state.vel(0), state.vel(1), x0(state_index.turn_rate);
        return x_comp;
    }
    Eigen::VectorXd get_ocp_state(const Eigen::VectorXd &x_pred, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        const double turn_angle = x_pred(2);
        const Eigen::Vector2d vel = x_pred.segment<2>(3);

        Eigen::Matrix2d T_body_world;
        T_body_world << std::cos(turn_angle), std::sin(turn_angle),
                       -std::sin(turn_angle), std::cos(turn_angle);

        Eigen::Vector2d vel_body = T_body_world * vel;
        vel_body(0) = std::max(0.501, vel_body(0)); // Project vx onto speed range (lower bound)
        vel_body(1) = std::max(-0.049, std::min(vel_body(1), 0.049)); // Project vy

        Eigen::VectorXd x_ocp(8);

        x_ocp(state_index.pos_x) = x_pred(0);
        x_ocp(state_index.pos_y) = x_pred(1);
        x_ocp(state_index.turn_angle) = x_pred(2);
        x_ocp(state_index.vel_x) = vel_body(0);
        x_ocp(state_index.vel_y) = vel_body(1);
        x_ocp(state_index.turn_rate) = x_pred(5);
        x_ocp(state_index.path_param) = x0(state_index.path_param);
        x_ocp(state_index.path_param_vel) = x0(state_index.path_param_vel);

        return x_ocp;
    }
    KinIntegrator_DelayCompensator() : DelayCompensator(DynamicsMode::CONTINUOUS, 0.005)
    {
    }
};

class DynamicBicycle_DelayCompensator : public DelayCompensator
{
public:
    using Model = DynamicBicycleWithTracking;
    using rkd5 = boost::numeric::odeint::runge_kutta_dopri5<Model::state_t<double>>;

    DynamicBicycleWithTracking model;
    rkd5 model_integrator;

    Eigen::VectorXd cont_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
        Model::state_t<double> x_t = x;
        Model::input_t<double> u_t = u;
        Model::param_t<double> p_t;
        (void) p_t;
        Eigen::VectorXd x_dot = model.dynamics_impl<double>(x_t, u_t, p_t);
        return x_dot;
    }
    Eigen::VectorXd discr_dynamics_impl(const Eigen::VectorXd &x, const Eigen::VectorXd &u) override
    {
        Model::state_t<double> x_t = x;
        Model::input_t<double> u_t = u;
        Model::param_t<double> p_t;
        (void) p_t;
        auto model_dynamics = [&](const Eigen::VectorXd &x, Eigen::Matrix<double, Model::nx, 1> &x_dot, double dt){
            (void) dt;
            x_dot = model.dynamics_impl<double>(x, u_t, p_t);
        };
        model_integrator.do_step(model_dynamics, x_t, 0.0, m_dt);
        return x_t;
    }
    Eigen::VectorXd compose_delayed_state(const RacingOcpParams::RacingState &state, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x0) override
    {
        (void) state;
        Eigen::VectorXd x_comp(8);

        x_comp(state_index.pos_x) = x0(state_index.pos_x);
        x_comp(state_index.pos_y) = x0(state_index.pos_y);
        x_comp(state_index.turn_angle) = x0(state_index.turn_angle);
        x_comp(state_index.vel_x) = x0(state_index.vel_x);
        x_comp(state_index.vel_y) = x0(state_index.vel_y);
        x_comp(state_index.turn_rate) = x0(state_index.turn_rate);
        x_comp(state_index.path_param) = x0(state_index.path_param);
        x_comp(state_index.path_param_vel) = x0(state_index.path_param_vel);

        return x_comp;
    }
    Eigen::VectorXd get_ocp_state(const Eigen::VectorXd &x_pred, const RacingOcpParams::StateIndex &state_index, const Eigen::VectorXd &x_0) override
    {
        (void) x_0;
        Eigen::VectorXd x_ocp(8);

        x_ocp(state_index.pos_x) = x_pred(state_index.pos_x);
        x_ocp(state_index.pos_y) = x_pred(state_index.pos_y);
        x_ocp(state_index.turn_angle) = x_pred(state_index.turn_angle);
        x_ocp(state_index.vel_x) = x_pred(state_index.vel_x);
        x_ocp(state_index.vel_y) = x_pred(state_index.vel_y);
        x_ocp(state_index.turn_rate) = x_pred(state_index.turn_rate);
        x_ocp(state_index.path_param) = x_pred(state_index.path_param);
        x_ocp(state_index.path_param_vel) = x_pred(state_index.path_param_vel);

        return x_ocp;
    }
    explicit DynamicBicycle_DelayCompensator(const DynamicsMode &dynamicsMode = DynamicsMode::CONTINUOUS) : DelayCompensator(dynamicsMode, 0.005)
    {
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
            delay_compensator = std::make_shared<DynamicBicycle_DelayCompensator>(DelayCompensator::DynamicsMode::CONTINUOUS);
            break;
        case 3:
            delay_compensator = std::make_shared<DynamicBicycle_DelayCompensator>(DelayCompensator::DynamicsMode::DISCRETE);
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
#endif //SRC_SPLINEFOLLOWINGTIREDELAYCOMPENSATION_HPP
