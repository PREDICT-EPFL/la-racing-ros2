#ifndef SRC_DRIVINGDYNAMICSCASADI_HPP
#define SRC_DRIVINGDYNAMICSCASADI_HPP

#include "model_common/common_utils.hpp"
#include "model_common/DynamicsBaseCasadi.hpp"
#include "model_common/QuaternionMathCasadi.hpp"

namespace driving_model {
namespace casadi_model {

using SX = casadi::SX;

class BicycleParams
{
public:
    SX g = SX::sym("g");
    SX rho = SX::sym("rho");

    SX wheelbase = SX::sym("wheelbase");
    SX l_r = SX::sym("l_r");

    SX steering_angle_offset = SX::sym("steering_angle_offset");
    SX steering_angle_gain = SX::sym("steering_angle_gain");
    SX steering_angle_TC = SX::sym("steering_angle_TC");

    SX speed_gain = SX::sym("speed_gain");
    SX speed_TC = SX::sym("speed_TC");


    std::map<std::string, double> bicycleDefaultParams{
            {"g",                     0.806},
            {"rho",                   1.225},

            {"wheelbase",             2.0},
            {"l_r",                   1.1},

            {"steering_angle_offset", 1.0 * M_PI / 180.0},
            {"steering_angle_gain",    15.0 * M_PI / 180.0},
            {"steering_angle_TC",     0.05},

            {"speed_gain",            10},
            {"speed_TC",              0.4},
    };
};

class KinematicBicycle : protected BicycleParams,
                         public model_common::casadi_model::DynamicsBase
{
public:
    explicit KinematicBicycle(const std::string &filename,
                              const std::map<std::string, double> &static_params = {},
                              const std::vector<std::string> &const_params = {},
                              const std::vector<std::string> &opt_params = {}) :
            DynamicsBase()
    {
        using namespace casadi;

        std::cout << "KinematicBicycle\n";

        x_default = DM::vertcat({0.2, 0.1, 20.0 * M_PI / 180.0});

        u_ubound = DM::vertcat({1, 1});
        u_lbound = DM::vertcat({-1, 1});

        x_trim_ubound = DM::inf(3, 1);
        x_trim_lbound = -x_trim_ubound;

        u_trim_ubound = DM::inf(2, 1);
        u_trim_lbound = u_trim_ubound;

        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        model_params = SX::vertcat({model_params,
                                    wheelbase, l_r,
                                    steering_angle_offset,
                                    speed_gain, steering_angle_gain});
        default_params.insert(bicycleDefaultParams.begin(), bicycleDefaultParams.end());

        /** ============================================================================================================ **/
        /** Model definition **/
        SX x = SX::sym("pos_x");
        SX y = SX::sym("pos_y");
        SX theta = SX::sym("theta");

        SX steering = SX::sym("steering");
        SX throttle = SX::sym("throttle");

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        SX steering_angle = steering_angle_offset + steering_angle_gain * steering;
        SX V = speed_gain * throttle;

        // Center of mass as reference point
        SX slip_angle = atan2(l_r * tan(steering_angle), wheelbase);

        SX x_dot = V * cos(slip_angle + theta);
        SX y_dot = V * sin(slip_angle + theta);
        SX theta_dot = V / l_r * sin(slip_angle);

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, const_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state = SX::vertcat({x, y, theta});
        SX input = SX::vertcat({steering, throttle});
        SX state_dot = SX::vertcat({x_dot, y_dot, theta_dot});

        SX vel_x = V * cos(theta);
        SX vel_y = V * sin(theta);
        SX output = SX::vertcat({x, y, theta, vel_x, vel_y, theta_dot});

        this->State = state;
        this->Control = input;
        this->StateDot = state_dot;
        this->Output = output;

        /** Additional output mappings **/
        SX jacobian_f_x = SX::jacobian(state_dot, state);
        SX jacobian_f_u = SX::jacobian(state_dot, input);

        SX dyn_out = SX::vertcat({theta_dot, V, debugSX});

        output_mappings.insert({{"V",            V},
                                {"dyn_out",      dyn_out},

                                {"jacobian_f_x", jacobian_f_x},
                                {"jacobian_f_u", jacobian_f_u},

                                {"debugSX",      debugSX}});
    }
};

class KinematicBicycleWithActuators : public KinematicBicycle
{
public:
    explicit KinematicBicycleWithActuators(const std::string &filename,
                                           const std::map<std::string, double> &static_params = {},
                                           const std::vector<std::string> &const_params = {},
                                           const std::vector<std::string> &opt_params = {}) :
            KinematicBicycle(filename, static_params, const_params, opt_params)
    {
        using namespace casadi;

        std::cout << "KinematicBicycleWithActuators\n";

        x_default = DM::vertcat({KinematicBicycle::x_default, 0, 0});

        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        model_params = SX::vertcat({steering_angle_TC, speed_TC});
        default_params.insert(bicycleDefaultParams.begin(), bicycleDefaultParams.end());

        /** ============================================================================================================ **/
        /** Model definition **/
        SX steering = this->Control(0);
        SX throttle = this->Control(1);

        SX steering_cmd = SX::sym("steering_cmd"); // -1 ... 1
        SX throttle_cmd = SX::sym("throttle_cmd"); // -1 ... 1

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /* Actuator dynamics */
        SX steering_dot = (steering_cmd - steering) / steering_angle_TC;
        SX throttle_dot = (throttle_cmd - throttle) / speed_TC;

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, const_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX aug_state = SX::vertcat({this->State, steering, throttle});
        SX input = SX::vertcat({steering_cmd, throttle_cmd});
        SX aug_state_dot = SX::vertcat({this->StateDot, steering_dot, throttle_dot});

        //SX output = this->Output; // Stays parent's output

        this->State = aug_state;
        this->Control = input;
        this->StateDot = aug_state_dot;
        //this->Output = output; // Stays parent's output
    }
};

//class KinematicBicycleDynamics : public model_common::casadi_model::DynamicsBase
//{
//public:
//    explicit FtBicyleDynamics(const std::string &filename,
//                              const std::map<std::string, double> &static_params = {},
//                              const std::vector<std::string> &dyn_params = {},
//                              const std::vector<std::string> &opt_params = {}, bool dummy_input = false) :
//            DynamicsBase()
//    {
//        using namespace casadi;
//
//        std::cout << "KinematicBicycleDynamics\n";
//
//        /** ============================================================================================================ **/
//        /** Model parameters **/
//        SX model_params;
//        std::map<std::string, double> default_params;
//
//        /** --------------------- **/
//        /** Environment & wind    **/
//        /** --------------------- **/
//        const double g = 9.806;   // gravitational acceleration [m/s2] [WGS84]
//
//        model_params = SX::vertcat({model_params}); /* dvW_N, dvW_E are added last (see below). ...
//    They will be added dynamically for each sequence, so they shall appear at the end of the param vector. */
//
//        /** --------------------- **/
//        /** Geometry              **/
//        /** --------------------- **/
//        SX l_fr = SX::sym("l_fr"); // Distance between CoG and front axle [m]
//        SX l_re = SX::sym("l_re"); // Distance between CoG and rear axle [m]
//
//        default_params.insert({"l_fr", 0.05});
//        default_params.insert({"l_re", 0.05});
//        model_params = SX::vertcat({model_params, l_fr, l_re});
//
//        /** --------------------------- **/
//        /** Mass and inertia            **/
//        /** --------------------------- **/
//        SX mass = SX::sym("mass");
//        SX Izz = SX::sym("Izz");
//
//        default_params.insert({"mass", 0.2});
//        default_params.insert({"Izz", 0.001});
//        model_params = SX::vertcat({model_params, mass, Izz});
//
//        /** ------------------------------- **/
//        /** Aerodynamic effects             **/
//        /** ------------------------------- **/
//        SX Cmt = SX::sym("Cmt");
//        SX Cmb = SX::sym("Cmb");
//        SX CD = SX::sym("CD");
//        SX F_resi = SX::sym("F_resi");
//
//        default_params.insert({"Cmt", 1});
//        default_params.insert({"Cmb", 1});
//        default_params.insert({"CD", 0.0001});
//        default_params.insert({"F_resi", 0.001});
//        model_params = SX::vertcat({model_params, Cmt, Cmb, CD, F_resi});
//
//        /** ------------------------------- **/
//        /** Tire parameters                 **/
//        /** ------------------------------- **/
//        SX B_fr = SX::sym("B_fr");
//        SX C_fr = SX::sym("C_fr");
//        SX D_fr = SX::sym("D_fr");
//
//        SX B_re = SX::sym("B_re");
//        SX C_re = SX::sym("C_re");
//        SX D_re = SX::sym("D_re");
//
//        default_params.insert({"B_fr", 0.1});
//        default_params.insert({"C_fr", 0.1});
//        default_params.insert({"D_fr", 0.1});
//        default_params.insert({"B_re", 0.1});
//        default_params.insert({"C_re", 0.1});
//        default_params.insert({"D_re", 0.1});
//        model_params = SX::vertcat({model_params, B_fr, C_fr, D_fr, B_re, C_re, D_re});
//
//        /** ------------------------------ **/
//        /**      Engine parameters         **/
//        /** ------------------------------ **/
//        SX accel_coeff = SX::sym("accel_coeff");
//        SX brake_coeff = SX::sym("brake_coeff");
//
//        default_params.insert({"accel_coeff", 1});
//        default_params.insert({"brake_coeff", 1});
//        model_params = SX::vertcat({model_params, accel_coeff, brake_coeff});
//
//        /** ============================================================================================================ **/
//        /** Start of model **/
//        SX debugSX;
//
//        /** State variables **/
//        /* Body frame: Front, left, up */
//        SX vx = SX::sym("vx");
//        SX vy = SX::sym("vy");
//        SX v = SX::vertcat({vx, vy}); // Linear velocity in body frame (front-left) frame [m/s]
//
//        SX r = SX::sym("r");     // yaw rate, Angular velocity (body frame) [rad/s]
//
//        SX posX = SX::sym("posX");
//        SX posY = SX::sym("posY");
//        SX pos = SX::vertcat({posX, posY}); // Position of the CoG (X-Y frame) [m]
//
//        SX yaw = SX::sym("yaw"); // Heading angle from global X (counter-clockwise)
//
//        /** Control variables **/
//        SX dR = SX::sym("dR"); // Steering angle (controls yaw rate r) (rad)
//        SX dT = SX::sym("dT"); // Throttle (0 ... 1)
//        SX dB = SX::sym("dB"); // Brake (0 ... 1)
//
//        /** ---------------------------------------------------------- **/
//        /** Thrust Forces and Moments in body frame **/
//        /** ---------------------------------------------------------- **/
//        SX accel_torque = accel_coeff * dT;
//        SX brake_torque = brake_coeff * dB;
//
//        /* Longitudinal force */
//        SX drag = CD * sq(vx);
//        SX Fx = Cmt * accel_torque - Cmb * brake_torque - F_resi - drag;
//
//        /* Lateral forces */
//        SX alpha_fr = atan((vy + l_fr * r) / vx);
//        SX alpha_re = atan((vy - l_re * r) / vx);
//
//        SX Ffy = D_fr * sin(C_fr * atan(B_fr * alpha_fr));
//        SX Fry = D_re * sin(C_re * atan(B_re * alpha_re));
//
//        SX b_F = SX::vertcat({Fx - Ffy * sin(dR),
//                              Fry + Ffy * cos(dR)});
//
//        SX Mtv = 0; // additional moment from torque vectoring
//
//        /** ---------------- **/
//        /** Motion equations **/
//        /** ---------------- **/
//        /** Linear motion equation (body frame) **/
//        SX spec_nongrav_force = b_F / mass;
//        SX v_dot = spec_nongrav_force + mass * SX::vertcat({vy, -vx}) * r;
//
//        /** Angular motion equation (body frame) **/
//        SX r_dot = (Ffy * l_fr * cos(dR) - Fry * l_re + Mtv) / Izz;
//
//        /** ------------------------------------ **/
//        /** Kinematic Equations (geodetic frame) **/
//        /** ------------------------------------ **/
//        /** Translation: position derivative **/
//        SX pos_dot = SX::vertcat({vx * cos(yaw) - vy * sin(yaw),
//                                  vy * sin(yaw) + vy * cos(yaw)});
////    SX pos_dot = SX::vertcat({0, 0}); // Fix position to investigate attitude dynamics
//
//        /** Rotation: attitude derivative **/
//        SX yaw_dot = r;
//
//        /** End of model **/
//        /** ============================================================================================================ **/
//
//        /** Categorize parameters **/
//        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);
//
//        /** State, Dynamics, Control **/
//        SX state = SX::vertcat({v, r, pos, yaw});
//        SX dynamics = SX::vertcat({v_dot, r_dot, pos_dot, yaw_dot});
//
//        x_default = DM::vertcat({1, 0.001,
//                                 2.0 * M_PI / 180.0,
//                                 0, 0,
//                                 30.0 * M_PI / 180.0});
//
//        DM ubx = DM::vertcat({DM::inf(), DM::inf(), DM::inf(),
//                              DM::inf(), DM::inf(), DM::inf()});
//        x_trim_ubound = ubx;
//        x_trim_lbound = -ubx;
//        SX control = SX::vertcat({dR, dT, dB});
//
//        u_ubound = DM::vertcat({15.0 * M_PI / 180.0, 1, 1});
//        u_lbound = DM::vertcat({-15.0 * M_PI / 180.0, 0, 0});
//
//        u_trim_ubound = u_ubound;
//        u_trim_lbound = u_lbound;
//
//        this->State = state;
//        this->Control = control;
//        this->Dynamics = dynamics;
//
//        /** Output mappings **/
//        SX jacobian_f_x = SX::jacobian(dynamics, state);
//        SX jacobian_f_u = SX::jacobian(dynamics, control);
//
//        /* Kinematic frame */
//        SX g_vg = pos_dot;
//        SX Vg = SX::norm_2(g_vg);
//        SX chi = atan2(g_vg(1), g_vg(0));
//
//        /* Full output (all in one vector) NOTE THAT debugSX only participates in vector if defined
//         * anywhere else than initial declaration */
//        SX sim_output = SX::vertcat({g_vg, Vg, chi, debugSX});
//
//        output_mappings.insert({{"Fx",                 Fx},
//                                {"Ftot",               norm_2(b_F)},
//                                {"alpha_fr",           alpha_fr},
//                                {"alpha_re",           alpha_re},
//
//                                {"Vg",                 Vg},
//                                {"chi",                chi},
//
//                                {"spec_nongrav_force", spec_nongrav_force},
//
//                                {"sim_output",         sim_output},
//
//                                {"jacobian_f_x",       jacobian_f_x},
//                                {"jacobian_f_u",       jacobian_f_u},
//
//                                {"debugSX",            debugSX}});
//    }
//protected:
//
//};

} //namespace casadi_model
} //namespace driving_model

#endif //SRC_DRIVINGDYNAMICSCASADI_HPP
