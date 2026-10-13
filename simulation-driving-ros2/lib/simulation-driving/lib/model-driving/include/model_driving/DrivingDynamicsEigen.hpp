#ifndef SRC_DRIVINGDYNAMICSEIGEN_HPP
#define SRC_DRIVINGDYNAMICSEIGEN_HPP

#include <iostream>
#include <iomanip>
#include <Eigen/Geometry>

#include "model_common/common_utils.hpp"
#include "model_common/DynamicsBaseEigen.hpp"
#include "model_common/ParamsBaseEigen.hpp"
#include "model_common/QuaternionMathEigen.hpp"

namespace driving_model {
namespace eigen_model {

using namespace model_common::eigen_model;

using Scalar = double;

struct BicycleParams : public ParamsBase<Scalar>
{
    StaticParam dummy{0};
    StaticParam g{9.806};           // -
    StaticParam rho{1.225};         // Air density in kg/m^3

    StaticParam mass{1100};         // Mass in kg
    StaticParam Izz{0.01};          // Moment of inertia around (vertical) z axis in kg/m^2

    StaticParam cross_area{0.0015}; // Front cross area
    StaticParam wheelbase{2.0};     // Distance between front and rear axle in m
    StaticParam l_r{1.1};           // Distance between rear axle and center of mass in m

    StaticParam c_friction{0.02};
    StaticParam c_drag{0.3};

    StaticParam Vr0{0};
    StaticParam CR0{0};
    StaticParam CRv{0};
    StaticParam c_res0{0};          // Rolling resistance for any nonzero speed (N)
    StaticParam c_resV{0};          // Rolling resistance constant on speed (N/(m/s))
    StaticParam c_resV2{0};         // Rolling resistance constant on speed squared (N/(m/s)^2)
    StaticParam c_resSt{0};         // Rolling resistance constant on steering (N/-)

    StaticParam steering_angle_offset{1.0 * M_PI / 180.0};
    StaticParam steering_angle_gain{10.0 * M_PI / 180.0};
    StaticParam steering_angle_TC{0.05};

    StaticParam speed_gain{10};
    StaticParam speed_TC{0.4};
    StaticParam speed_ctrl_FF{0.4};
    StaticParam speed_ctrl_P{0.01};

    StaticParam throttle_TC{0.01};   // Time constant for 1st order throttle dynamics
    StaticParam c_Vcmd{10};          // Speed gain (speed command coefficient)
    StaticParam FF_Vcmd{0};          // Thr -> force feedforward
    StaticParam FF_thr2{0};          // Thr -> force feedforward exp x scaling
    StaticParam FF_thr3{0};          // Thr -> force feedforward exp x offset
    StaticParam P_V{0};              // Proportional gain on speed error

    StaticParam acceleration_max{18.0};

    StaticParam powertrain_a_max{11.0};
    StaticParam powertrain_a_neg_max{5.5};

    StaticParam turb_ref_length{};

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        using namespace model_common;
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        get_as_double(yamlNode, "mass", mass);
        get_as_double(yamlNode, "Izz", Izz);

        get_as_double(yamlNode, "cross_area", cross_area);
        get_as_double(yamlNode, "wheelbase", wheelbase);
        get_as_double(yamlNode, "l_r", l_r);

        get_as_double(yamlNode, "c_friction", c_friction);
        get_as_double(yamlNode, "c_drag", c_drag);

        get_as_double(yamlNode, "Vr0", Vr0);
        get_as_double(yamlNode, "CR0", CR0);
        get_as_double(yamlNode, "CRv", CRv);
        get_as_double(yamlNode, "c_res0", c_res0);
        get_as_double(yamlNode, "c_resV", c_resV);
        get_as_double(yamlNode, "c_resV2", c_resV2);
        get_as_double(yamlNode, "c_resSt", c_resSt);

        get_as_double(yamlNode, "steering_angle_offset", steering_angle_offset);

        get_as_double(yamlNode, "steering_angle_gain", steering_angle_gain);
        get_as_double(yamlNode, "steering_angle_TC", steering_angle_TC);

        get_as_double(yamlNode, "speed_gain", speed_gain);
        get_as_double(yamlNode, "speed_TC", speed_TC);

        get_as_double(yamlNode, "acceleration_max", acceleration_max);

        get_as_double(yamlNode, "powertrain_a_max", powertrain_a_max);
        get_as_double(yamlNode, "powertrain_a_neg_max", powertrain_a_neg_max);

        get_as_double(yamlNode, "throttle_TC", throttle_TC);
        get_as_double(yamlNode, "c_Vcmd", c_Vcmd);
        get_as_double(yamlNode, "FF_Vcmd", FF_Vcmd);
        get_as_double(yamlNode, "FF_thr2", FF_thr2);
        get_as_double(yamlNode, "FF_thr3", FF_thr3);
        get_as_double(yamlNode, "P_V", P_V);

        get_as_double(yamlNode, "turb_ref_length", turb_ref_length);
    }
};

struct TireParams : public ParamsBase<Scalar>
{
    StaticParam Bf{0.001};
    StaticParam Cf{0.001};
    StaticParam Df{0.001};

    StaticParam Br{0.001};
    StaticParam Cr{0.001};
    StaticParam Dr{0.001};

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        using namespace model_common;
        const YAML::Node yamlNode = YAML::LoadFile(yaml_filepath);

        Bf = get_value(yamlNode, "Bf");
        Cf = get_value(yamlNode, "Cf");
        Df = get_value(yamlNode, "Df");

        Br = get_value(yamlNode, "Br");
        Cr = get_value(yamlNode, "Cr");
        Dr = get_value(yamlNode, "Dr");
    }

};

class KinematicBicycle : public BicycleParams,
                         public DynamicsBase<Scalar,
                                 /* nx */ 3, /* nu */ 2, /* np */ 0, /* ny */ 6, /* nz */ 2>
{
public:
    KinematicBicycle()
    {
        // Default state
        x_default << 0.2, 0.1, 20.0 * M_PI / 180.0; // x, y, theta, V

        // Physical control bounds
        u_ubound << 1, 1;
        u_lbound << -1, -1;

        // Trim bounds - state
        x_trim_ubound.setConstant(INFINITY);
        x_trim_lbound = -x_trim_ubound;

        // Trim bounds - not defined
        u_trim_ubound.setConstant(INFINITY);
        u_trim_lbound = -u_trim_ubound;
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params(1);
        const_params << rho;
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State variables **/
        scalar_t x = state(0);
        scalar_t y = state(1);
        scalar_t theta = state(2);

        /** Control variables **/
        scalar_t steering_angle = steering_angle_offset + steering_angle_gain * input(0);
        scalar_t V = speed_gain * input(1);

        // Avoid unused compiler warning
        (void) x;
        (void) y;
        (void) param;

        /** ============================================================================================================ **/
        /** Start of model **/

        // Center of mass as reference point
        scalar_t slip_angle = std::atan(l_r / wheelbase * std::tan(steering_angle));

        scalar_t x_dot = V * std::cos(slip_angle + theta);
        scalar_t y_dot = V * std::sin(slip_angle + theta);
        scalar_t theta_dot = V / wheelbase * std::cos(slip_angle) * std::tan(steering_angle);

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<scalar_t> state_dot;
        state_dot << x_dot, y_dot, theta_dot;
        dyn_out << theta_dot, V;

        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const Eigen::Ref<const state_t<scalar_t>> &state,
                              const Eigen::Ref<const input_t<scalar_t>> &input,
                              const Eigen::Ref<const param_t<scalar_t>> &param) const
    {
        (void) param;

        scalar_t pos_x = state(0);
        scalar_t pos_y = state(1);
        scalar_t turn_angle = state(2);

        scalar_t steering_angle = steering_angle_offset + steering_angle_gain * input(0);
        scalar_t V = speed_gain * input(1);
        scalar_t slip_angle = atan2(l_r * tan(steering_angle), static_cast<scalar_t>(wheelbase));

        scalar_t vel_x = V * cos(turn_angle);
        scalar_t vel_y = V * sin(turn_angle);
        scalar_t turn_rate = V / l_r * sin(slip_angle);

        output_t<scalar_t> output;
        output << pos_x, pos_y, turn_angle, vel_x, vel_y, turn_rate;
        return output;
    }
};
class KinematicBicycleWithActuators : public BicycleParams,
                                      public DynamicsBase<Scalar,
                                              /* nx */ KinematicBicycle::nx + 2,
                                              /* nu */ KinematicBicycle::nu,
                                              /* np */ KinematicBicycle::np,
                                              /* ny */ KinematicBicycle::ny,
                                              /* nz */ KinematicBicycle::nz>
{
public:
    using KinBicycle = KinematicBicycle;

    KinematicBicycleWithActuators()
    {
        x_default << kinBicycle.get_default_initial_state(), 0, 0;

        u_ubound = kinBicycle.u_ubound;
        u_lbound = kinBicycle.u_lbound;
    }

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        BicycleParams::load_params_from_yaml(yaml_filepath);
        // Need to update params in member instance, too
        kinBicycle.load_params_from_yaml(yaml_filepath);
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        kinBicycle.set_const_params(const_params);
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params = kinBicycle.get_const_params();
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State, Control, Parameter parsing **/
        const int nx_aug = nx - KinBicycle::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(KinBicycle::nx);

        scalar_t steering = aug_state(0); // -1 ... 1
        scalar_t throttle = aug_state(1); // -1 ... 1

        scalar_t steering_cmd = input(0); // -1 ... 1
        scalar_t throttle_cmd = input(1); // -1 ... 1

        /** ============================================================================================================ **/
        /** Start of model **/
        /* Actuator dynamics */
        scalar_t steering_dot = (steering_cmd - steering) / steering_angle_TC;
        scalar_t throttle_dot = (throttle_cmd - throttle) / speed_TC;

        /** End of model **/
        /** ============================================================================================================ **/
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state_dot;
        aug_state_dot << steering_dot, throttle_dot;

        /** Evaluate un-augmented dynamics, append augmented dynamics **/
        KinBicycle::state_t<scalar_t> base_state = state.template segment<KinBicycle::nx>(0);
        KinBicycle::input_t<scalar_t> base_input = aug_state;
        KinBicycle::state_t<scalar_t> base_state_dot = kinBicycle.dynamics<scalar_t>(base_state, base_input, param,
                                                                                     dyn_out);

        state_t<scalar_t> state_dot;
        state_dot << base_state_dot, aug_state_dot;
        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const Eigen::Ref<const state_t<scalar_t>> &state,
                              const Eigen::Ref<const input_t<scalar_t>> &input,
                              const Eigen::Ref<const param_t<scalar_t>> &param) const
    {
        (void) input;

        KinBicycle ::state_t<scalar_t> base_state = state.template segment<KinBicycle::nx>(0);

        const int nx_aug = nx - KinBicycle::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(KinBicycle::nx);

        KinBicycle::output_t<scalar_t> base_output = kinBicycle.output<scalar_t>(base_state, aug_state, param);
        return base_output;
    }

private:
    KinBicycle kinBicycle;
};

class AcceleratedBicycle : public BicycleParams,
                           public DynamicsBase<Scalar,
                                   /* nx */ KinematicBicycle::nx + 1,
                                   /* nu */ KinematicBicycle::nu,
                                   /* np */ KinematicBicycle::np,
                                   /* ny */ KinematicBicycle::ny,
                                   /* nz */ KinematicBicycle::nz>
{
public:
    using KinBicycle = KinematicBicycle;

    AcceleratedBicycle()
    {
        x_default << kinBicycle.get_default_initial_state(), 0.1; // V

        u_ubound << kinBicycle.u_ubound(0), 1; // lon_acc
        u_lbound << kinBicycle.u_lbound(0), -1;
    }

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        BicycleParams::load_params_from_yaml(yaml_filepath);
        // Need to update params in member instance, too
        kinBicycle.load_params_from_yaml(yaml_filepath);
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        kinBicycle.set_const_params(const_params);
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params = kinBicycle.get_const_params();
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State, Control, Parameter parsing **/
        const int nx_aug = nx - KinBicycle::nx;

        scalar_t V = state(3);
        scalar_t Vref = speed_gain * input(1);

        /** ============================================================================================================ **/
        /** Start of model **/

        scalar_t F_powertrain = (Vref - V) / speed_TC; // Emulate P controller in ESC
        const scalar_t F_powertrain_max = powertrain_a_max * mass;
        const scalar_t F_powertrain_brake_max = powertrain_a_neg_max * mass;
        F_powertrain = std::max(-F_powertrain_brake_max, std::min(F_powertrain, F_powertrain_max));

        scalar_t rolling_activation = 2.0 / M_PI * std::atan(5.0 * V);
        scalar_t rolling_resistance = CRv * std::abs(V) + CR0;         // CRv * std::abs(vx - Vr0) + CR0

        scalar_t dyn_pressure = rho / 2.0 * V * std::abs(V);
        scalar_t Drag = dyn_pressure * cross_area * c_drag;

        scalar_t Fx = F_powertrain - rolling_activation * rolling_resistance - Drag;

        //std::cout << "prop: " << lon_acc << ", res: " << rolling_resistance << ", Drag: " << Drag << "\n";
        scalar_t V_dot = Fx / mass;

        /** End of model **/
        /** ============================================================================================================ **/
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state_dot;
        aug_state_dot << V_dot;

        /** Evaluate un-augmented dynamics, append augmented dynamics **/
        KinBicycle ::state_t<scalar_t> base_state = state.template segment<KinBicycle::nx>(0);

        KinBicycle::input_t<scalar_t> base_input;
        base_input << input(0), V / speed_gain; // -1 ... 1

        KinBicycle::state_t <scalar_t> base_state_dot = kinBicycle.dynamics<scalar_t>(base_state, base_input, param,
                                                                                      dyn_out);

        state_t<scalar_t> state_dot;
        state_dot << base_state_dot, aug_state_dot;
        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const state_t<scalar_t> &state,
                              const input_t<scalar_t> &input,
                              const param_t<scalar_t> &param) const
    {
        KinBicycle::state_t<scalar_t> base_state = state.template segment<KinBicycle::nx>(0);

        KinBicycle::input_t<scalar_t> base_input;
        base_input << input(0), state(3) / speed_gain;

        KinBicycle::output_t<scalar_t> base_output = kinBicycle.output<scalar_t>(base_state, base_input, param);
        return base_output;
    }

private:
    KinBicycle kinBicycle;
};
class AcceleratedBicycleWithActuators : public BicycleParams,
                                        public DynamicsBase<Scalar,
                                                /* nx */ AcceleratedBicycle::nx + 2,
                                                /* nu */ AcceleratedBicycle::nu,
                                                /* np */ AcceleratedBicycle::np,
                                                /* ny */ AcceleratedBicycle::ny,
                                                /* nz */ AcceleratedBicycle::nz>
{
public:
    using AccBicycle = AcceleratedBicycle;

    AcceleratedBicycleWithActuators()
    {
        x_default << accBicycle.get_default_initial_state(), 0, 0;

        u_ubound = accBicycle.u_ubound;
        u_lbound = accBicycle.u_lbound;
    }

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        BicycleParams::load_params_from_yaml(yaml_filepath);
        // Need to update params in member instance, too
        accBicycle.load_params_from_yaml(yaml_filepath);
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        accBicycle.set_const_params(const_params);
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params = accBicycle.get_const_params();
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State, Control, Parameter parsing **/
        const int nx_aug = nx - AccBicycle ::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(AccBicycle ::nx);

        scalar_t steering = aug_state(0); // -1 ... 1
        scalar_t throttle = aug_state(1); // -1 ... 1

        scalar_t steering_cmd = input(0); // -1 ... 1
        scalar_t throttle_cmd = input(1); // -1 ... 1

        /** ============================================================================================================ **/
        /** Start of model **/
        /* Actuator dynamics */
        scalar_t steering_dot = (steering_cmd - steering) / steering_angle_TC;
        scalar_t throttle_dot = (throttle_cmd - throttle) / (speed_TC / 10.0); // Assume acceleration builds 10x faster

        /** End of model **/
        /** ============================================================================================================ **/
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state_dot;
        aug_state_dot << steering_dot, throttle_dot;

        /** Evaluate un-augmented dynamics, append augmented dynamics **/
        AccBicycle::state_t<scalar_t> base_state = state.template segment<AccBicycle::nx>(0);
        AccBicycle::input_t<scalar_t> base_input = aug_state;
        AccBicycle::state_t<scalar_t> base_state_dot = accBicycle.dynamics<scalar_t>(base_state, base_input, param,
                                                                                     dyn_out);

        state_t<scalar_t> state_dot;
        state_dot << base_state_dot, aug_state_dot;
        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const Eigen::Ref<const state_t<scalar_t>> &state,
                              const Eigen::Ref<const input_t<scalar_t>> &input,
                              const Eigen::Ref<const param_t<scalar_t>> &param) const
    {
        (void) input;

        AccBicycle::state_t<scalar_t> base_state = state.template segment<AccBicycle::nx>(0);

        const int nx_aug = nx - AccBicycle::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(AccBicycle::nx);

        AccBicycle::output_t<scalar_t> base_output = accBicycle.output<scalar_t>(base_state, aug_state, param);
        return base_output;
    }

private:
    AccBicycle accBicycle;
};

template<typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

/* Force/Torque Bicycle */
struct FtBicycleParams : BicycleParams, TireParams
{
    void load_params_from_yaml(const std::string &yaml_filepath)
    {
        BicycleParams::load_params_from_yaml(yaml_filepath);
        TireParams::load_params_from_yaml(yaml_filepath);
    }
};
class FtBicycle : public FtBicycleParams,
                  public DynamicsBase<Scalar,
                          /* nx */ 6, /* nu */ 2, /* np */ 0, /* ny */ 6, /* nz */ 4>
{
public:
    FtBicycle()
    {
        // Default state
        x_default << 0.2, 0.1, 20.0 * M_PI / 180.0, 1.0, 0.0, 0.0 * M_PI / 180.0; // x, y, phi, vx, vy, omega

        // Physical control bounds
        u_ubound << 1, 1;
        u_lbound << -1, -1;

        // Trim bounds - state
        x_trim_ubound.setConstant(INFINITY);
        x_trim_lbound = -x_trim_ubound;

        // Trim bounds - not defined
        u_trim_ubound.setConstant(INFINITY);
        u_trim_lbound = -u_trim_ubound;
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params(1);
        const_params << rho;
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State variables **/
        scalar_t pos_x = state(0);
        scalar_t pos_y = state(1);
        scalar_t phi = state(2);
        scalar_t vx = state(3);
        scalar_t vy = state(4);
        scalar_t omega = state(5);

        /** Control variables **/
        scalar_t steering = input(0);
        scalar_t throttle = input(1);

        // Avoid unused compiler warning
        (void) pos_x;
        (void) pos_y;
        (void) param;
//        (void) dyn_out;

        StaticParam l_f = wheelbase - l_r;

        /** ============================================================================================================ **/
        /** Start of model **/

        int p = 3;
        std::cout << std::fixed << std::setprecision(p);
        const Eigen::Vector2d v(vx, vy);
//        if (std::abs(v.norm() - std::sqrt(vx*vx + vy*vy)) > 0.001) { std::cout << "norm problem\n"; }
//        scalar_t F_powertrain = (vx_ref - vx) / speed_TC; // Emulate P controller in ESC

//        scalar_t vx_ref = speed_gain * input(1);
//        const scalar_t F_ff = speed_ctrl_FF * vx_ref;
//        const scalar_t speed_err = vx_ref - vx;
//        const scalar_t F_P = speed_ctrl_P * speed_err;
//        scalar_t F_powertrain = F_ff + F_P;
//
//        const scalar_t F_powertrain_max = powertrain_a_max * mass;
//        const scalar_t F_powertrain_brake_max = powertrain_a_neg_max * mass;
//        F_powertrain = std::max(-F_powertrain_brake_max, std::min(F_powertrain, F_powertrain_max));
//
        const scalar_t rolling_activation = 2.0 / M_PI * std::atan(5.0 * vx);
//        const scalar_t rolling_resistance = CRv * std::abs(vx) + CR0;         // CRv * std::abs(vx - Vr0) + CR0

        // Matlab: CarSpeedModel2
        const scalar_t V_cmd = c_Vcmd * throttle;

        auto thr_fn = [this] (const scalar_t &thr_) { return FF_Vcmd * std::exp(FF_thr2 * thr_ + FF_thr3); };
        auto thr_fn2 = [this, thr_fn] (const scalar_t &thr_) { return thr_fn(thr_) - thr_fn(0); };

        const scalar_t F_pwr_FFcmd = // FF_Vcmd * V_cmd + FF_thr2 * throttle * std::abs(throttle);
                                 ((V_cmd < 0) ? -1.0 : 1.0) * thr_fn2( std::abs(V_cmd) );

        const scalar_t V_err = V_cmd - vx;
        const scalar_t F_pwr_FB = P_V * V_err;

        const scalar_t F_pwr = F_pwr_FFcmd + F_pwr_FB;
        const scalar_t F_res = rolling_activation * c_res0 + c_resV * vx + c_resSt * std::abs(steering);

//        const scalar_t F_tot = F_pwr - rolling_activation * F_res;
        const scalar_t F_tot = F_pwr - F_res;

        const scalar_t Fx = F_tot;
//        //// TODO: Debug
//        std::cout << "V_cmd: " << V_cmd << ", F_pwr_FFcmd: " << F_pwr_FFcmd << ", F_pwr_FB: " << F_pwr_FB <<
//        ", F_pwr: " << F_pwr << ", F_res: " << F_res << ", F_drag: " << 0 << ", F_tot: " << F_tot << "\n";

//        const scalar_t dyn_pressure = rho / 2.0 * vx * std::abs(vx);
//        const scalar_t Drag = dyn_pressure * cross_area * c_drag;
//
//        const scalar_t Fx = F_powertrain - rolling_activation * rolling_resistance - Drag;

        // Local velocities at front and rear axle
        scalar_t steering_angle = steering_angle_offset + steering_angle_gain * input(0); // Steering

        const Eigen::Vector2d v_f(vx, vy + l_f * omega);
        const Eigen::Vector2d v_r(vx, vy - l_r * omega);
        const scalar_t vel_div_f = v_f(1) / (v_f(0) + 1e-5);
        const scalar_t vel_div_r = v_r(1) / (v_r(0) + 1e-5);
        scalar_t slip_angle_f = -std::atan(vel_div_f) + steering_angle; //vy + l_f * omega, vx + 1e-8) + steering_angle;
        scalar_t slip_angle_r = -std::atan(vel_div_r); //vy - l_r * omega, vx + 1e-8);

        // When going slow, diminish slip angle
        scalar_t slip_activation_f = 2.0 / M_PI * std::atan(10.0 * v_f.norm());
        scalar_t slip_activation_r = 2.0 / M_PI * std::atan(10.0 * v_r.norm());
        slip_activation_f *= slip_activation_f; // Squared
        slip_activation_r *= slip_activation_r; // Squared

        slip_angle_f *= slip_activation_f;
        slip_angle_r *= slip_activation_r;

        scalar_t Ffy = Df * std::sin(Cf * std::atan(Bf * slip_angle_f));
        scalar_t Fry = Dr * std::sin(Cr * std::atan(Br * slip_angle_r));
        // When going backwards, invert forces
        if (vx < 0.0)
        {
            Ffy = -Ffy;
            Fry = -Fry;
        }
        const scalar_t M_tv = 0;

        // Similar to 'Learning-based Model Predictive Control for Autonomous Racing'
        // Juraj Kabzan1, Lukas Hewing1, Alexander Liniger2, and Melanie N. Zeilinger1
        const scalar_t vx_dot = (Fx - Ffy * std::sin(steering_angle)) / mass + vy * omega;
        const scalar_t vy_dot = (Fry + Ffy * std::cos(steering_angle)) / mass - vx * omega;

        const scalar_t M_front = Ffy * std::cos(steering_angle) * l_f;
        const scalar_t M_rear = -Fry * l_r;
        const scalar_t omega_dot = (M_front + M_rear + M_tv) / Izz;

        // Center of mass as reference point
        const scalar_t pos_x_dot = vx * std::cos(phi) - vy * std::sin(phi);
        const scalar_t pos_y_dot = vx * std::sin(phi) + vy * std::cos(phi);
        const scalar_t phi_dot = omega;

//        int w = p + 3;
//        std::cout << std::right
//                  << "v: (" << std::setw(w) << vx << ", " << std::setw(w) << vy << "), " << std::setw(w) << omega << ", "
//                  << "vx ref: " << std::setw(w) << vx_ref << ", "
//                  << "P: " << std::setw(w) << F_powertrain << ", R: " << std::setw(w) << rolling_resistance << ", act: " << std::setw(w) << rolling_activation << ", D:" << std::setw(w) << Drag << ", "
//                  << "Fx: " << std::setw(w) << Fx << ", "
                //                << "V: " << V
//                << ", Fpwr: " << F_powertrain
                //                  << ", act: " << rolling_activation
//                << ", act*R: " << rolling_activation * rolling_resistance
//                  << "vy (f: " << std::setw(w + 1) << vy_f << ", " << std::setw(w + 1) << vy_r << "), "
//                  << "div (f: " << std::setw(w + 1) << vel_div_f << ", " << std::setw(w + 1) << vel_div_r << "), "
//                  << "slip (f: " << std::setw(w + 1) << slip_angle_f * 180.0 / M_PI << ", r: " << std::setw(w + 1) << slip_angle_r * 180.0 / M_PI << ")º, "
                //                  << ", Ffy: " << Ffy
                //                  << ", Fry: " << Fry
//                  << "Fy (f; " << std::setw(w) << Ffy << ", r: " << std::setw(w) << Fry << "), "
//                  << "M (f: " << std::setw(w) << M_front << ", r: " << std::setw(w) << M_rear << ")"
//                  << "\n";

//        std::cout << std::setprecision(6)
//                  << "Front slip: " << slip_angle_f * 180. / M_PI << ", "
//                  << "Rear slip: " << slip_angle_r * 180. / M_PI << "\n";

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<scalar_t> state_dot;
        state_dot << pos_x_dot, pos_y_dot, phi_dot, vx_dot, vy_dot, omega_dot;

        dyn_out << vel_div_f, vel_div_r, slip_angle_f, slip_angle_r;
        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const state_t<scalar_t> &state,
                              const input_t<scalar_t> &input,
                              const param_t<scalar_t> &param) const
    {
        (void) input;
        (void) param;

        Eigen::Matrix<scalar_t, 2, 1> b_vel = state.template segment<2>(3);
        scalar_t phi = state(2);

        // Transform velocity from body to world frame
        Eigen::Matrix2d T_wb;
        T_wb << std::cos(-phi), std::sin(-phi),
                -std::sin(-phi), std::cos(-phi);
        const Eigen::Vector2d w_vel = T_wb * b_vel;

        output_t<scalar_t> output;
        output << state.template segment<3>(0), w_vel, state(5);
//        std::cout << "output: " << output.transpose() << "\n";
        return output;
    }
};

class FtBicycleWithActuators : public BicycleParams,
                               public DynamicsBase<Scalar,
                                       /* nx */ FtBicycle::nx + 2,
                                       /* nu */ FtBicycle::nu,
                                       /* np */ FtBicycle::np,
                                       /* ny */ FtBicycle::ny,
                                       /* nz */ FtBicycle::nz>
{
public:
    FtBicycleWithActuators()
    {
        x_default << ftBicycle.get_default_initial_state(), 0, 0;

        u_ubound = ftBicycle.u_ubound;
        u_lbound = ftBicycle.u_lbound;
    }

    void load_params_from_yaml(const std::string &yaml_filepath) override
    {
        BicycleParams::load_params_from_yaml(yaml_filepath);
        // Need to update params in member instance, too
        ftBicycle.load_params_from_yaml(yaml_filepath);
    }

    void set_const_params(const Eigen::VectorXd &const_params)
    {
        ftBicycle.set_const_params(const_params);
        if (const_params.size() > 0) { rho = const_params(0); }
    }
    Eigen::VectorXd get_const_params() const
    {
        Eigen::VectorXd const_params = ftBicycle.get_const_params();
        return const_params;
    }

    template<typename scalar_t>
    state_t<scalar_t> dynamics(const Eigen::Ref<const state_t<scalar_t>> &state,
                               const Eigen::Ref<const input_t<scalar_t>> &input,
                               const Eigen::Ref<const param_t<scalar_t>> &param,
                               Eigen::Ref<dynout_t<scalar_t>> dyn_out) const
    {
        /** State, Control, Parameter parsing **/
        const int nx_aug = nx - FtBicycle ::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(FtBicycle ::nx);

        scalar_t steering = aug_state(0); // -1 ... 1
        scalar_t throttle = aug_state(1); // -1 ... 1

        scalar_t steering_cmd = input(0); // -1 ... 1
        scalar_t throttle_cmd = input(1); // -1 ... 1

        /** ============================================================================================================ **/
        /** Start of model **/
        /* Actuator dynamics */
        scalar_t steering_dot = (steering_cmd - steering) / steering_angle_TC;
        scalar_t throttle_dot = (throttle_cmd - throttle) / throttle_TC;

        /** End of model **/
        /** ============================================================================================================ **/
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state_dot;
        aug_state_dot << steering_dot, throttle_dot;

        /** Evaluate un-augmented dynamics, append augmented dynamics **/
        FtBicycle::state_t<scalar_t> base_state = state.template segment<FtBicycle::nx>(0);
        FtBicycle::input_t<scalar_t> base_input = aug_state;
        FtBicycle::state_t<scalar_t> base_state_dot = ftBicycle.dynamics<scalar_t>(base_state, base_input, param,
                                                                                     dyn_out);

        state_t<scalar_t> state_dot;
        state_dot << base_state_dot, aug_state_dot;
        return state_dot;
    }

    template<typename scalar_t>
    output_t<scalar_t> output(const Eigen::Ref<const state_t<scalar_t>> &state,
                              const Eigen::Ref<const input_t<scalar_t>> &input,
                              const Eigen::Ref<const param_t<scalar_t>> &param) const
    {
        (void) input;

        FtBicycle::state_t<scalar_t> base_state = state.template segment<FtBicycle::nx>(0);

        const int nx_aug = nx - FtBicycle::nx;
        Eigen::Matrix<scalar_t, nx_aug, 1> aug_state = state.template segment<nx_aug>(FtBicycle::nx);

        FtBicycle::output_t<scalar_t> base_output = ftBicycle.output<scalar_t>(base_state, aug_state, param);
        return base_output;
    }

private:
    FtBicycle ftBicycle;
};

} //namespace eigen_model
} //namespace driving_model

#endif //SRC_DRIVINGDYNAMICSEIGEN_HPP
