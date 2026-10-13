#ifndef SRC_DRIVINGSIMULATOR_HPP
#define SRC_DRIVINGSIMULATOR_HPP

#include <Eigen/Core>

#include "simulation_common/SimulatorBase.hpp"
#include "simulation_common/sim_engine/SimEngineBase.hpp"

#include "simulation_common/sim_engine/SimEngineEigen.hpp"
#include "simulation_common/sim_engine/OdeIntPlant.hpp"

#include "simulation_common/SensorModel.hpp"

#ifdef WITH_CASADI
#include "casadi/casadi.hpp"
#include "simulation_common/sim_engine/SimEngineCasadi.hpp"
#endif

namespace simulation {
namespace driving {

class DrivingSimulator : public virtual SimulatorBase
{
public:
    template<typename EigenVehicle>
    void setup_eigen_vehicle(const std::string &model_params_file = "")
    {
        std::cout << "[Simulator]: Eigen sim engine (Boost ODE integration)\n";
        std::cout << "[Simulator]: Using model parameter file: " << model_params_file << "\n";

        /** Eigen model **/
        /* Copy pointer after setting up temporary instance */
        using EigenPlant = odeint_model::OdeIntPlant<EigenVehicle>;
        const auto sim_engine_ptr = std::make_shared<engine::EigenSimEngine<EigenPlant>>(get_sim_dt());
        if (!model_params_file.empty()) { sim_engine_ptr->plant.load_params_from_yaml(model_params_file); }
//        sim_engine_ptr->plant.set_static_params(staticParams.vec());
        m_sim_engine = sim_engine_ptr;

        /** Initialization **/
        m_state = sim_engine_ptr->plant.get_default_initial_state();
        m_control = m_control_in = Eigen::VectorXd::Zero(EigenVehicle::nu);
        m_params = {};
        m_sim_engine->initialize(m_state, m_control, m_params);
        m_output = m_sim_engine->get_output();

        const EigenVehicle vehicle;
        p_control_lb = vehicle.u_lbound;
        p_control_ub = vehicle.u_ubound;
    }
#ifdef WITH_CASADI
    template<typename CasadiVehicle>
    void setup_casadi_vehicle(const std::string &model_params_file = "")
    {
        std::cout << "[Simulator]: Casadi sim engine (CVODES ODE integration)\n";
        std::cout << "[Simulator]: Using model parameter file: " << model_params_file << "\n";

        /** Casadi model **/
        /* Construct kite dynamics for forward integration (for initial guess) */
        auto dynamics_model = CasadiVehicle(model_params_file);

        const casadi::Function dynamics_func = dynamics_model.getNumericDynamics(true, false);
        const casadi::Function output_func = dynamics_model.getNumericOutput(true, false);
        const casadi::Function dynOut_func = dynamics_model.getNumericMapping("dyn_out", true, false);

        m_sim_engine = std::make_shared<engine_casadi::SimEngineCasadi>(dynamics_func, get_sim_dt(),
                                                                        output_func, dynOut_func);

        /** Initialization **/
        const int n_aug = (p_sim_opts.actuator_dynamics ? dynamics_model.getNumericDynamics().size1_in(1) : 0);
        m_state = eigen_casadi_conversion::toEigenX(dynamics_model.getDefaultInitialState());
        m_control = m_control_in = Eigen::VectorXd::Zero(dynamics_model.getSymbolicControl().size1());
        m_params = {};
        m_sim_engine->initialize(m_state, m_control, m_params);
        m_output = m_sim_engine->get_output();

        p_control_lb = eigen_casadi_conversion::toEigenX(dynamics_model.u_lbound);
        p_control_ub = eigen_casadi_conversion::toEigenX(dynamics_model.u_ubound);
    }
#endif

    void set_actuator_rate(double actuator_rate)
    {
        p_sim_opts.actuator_dt = floor(1.0 / actuator_rate * 1e4) / 1e4; // Round to 0.1 ms
        if (m_VERBOSE >= PrintLevel1) std::cout << "p_sim_opts.actuator_dt: " << p_sim_opts.actuator_dt << "\n";
    }

    sensor_models::SimpleSensor3D& sensor_pose2d() { return m_sensor_pose2d; }
    sensor_models::SimpleSensor3D& sensor_vel() { return m_sensor_vel; }

    void set_partial_state(const Eigen::VectorXd &state, int start_index = 0)
    {
        if (state.size() > 0)
        {
            m_state.segment(start_index, state.size()) = state;
            m_sim_engine->set_state(m_state);
            //std::cout << "initial state: " << m_sim_engine->get_state() << "\n";
        }
    }
    void set_partial_control(const Eigen::VectorXd &control, int start_index = 0)
    {
        if (control.size() > 0)
        {
            Eigen::VectorXd full_control = m_control;
            full_control.segment(start_index, control.size()) = control;

            // Don't write to m_control directly because DrivingSimulator uses an additional control_in layer
            SimulatorBase::set_control(full_control);
            m_control_in = full_control;
        }
    }
    void set_control(const Eigen::VectorXd &control, unsigned control_id = 0) // Need to hide set_control from base class
    {
        // Need to hide set_control from base class because DrivingSimulator uses an additional control_in layer
        //SimulatorBase::set_control(control);
        m_control_in = control;
        m_control_id_in = control_id;
    }

    void init_wrapper()
    {
        init_impl();
    }

    struct DrivingSimOptions
    {
        bool actuator_dynamics{false};
        double actuator_dt{1.0 / 50.0};
    } p_sim_opts{};

    unsigned get_control_id() const { return m_control_id; }

protected:
    /** Execution **/
    void init_impl() override
    {
        m_sim_time_actuator_update = 0;
        update_control();

        std::cout << "\n[Simulator]: Initializing simulation ...\n";

        std::cout << "[Simulator]: Actuator dynamics " << (p_sim_opts.actuator_dynamics ? "ON" : "OFF") << "\n";
        std::cout << "[Simulator]: Actuator rate: ";
        if (p_sim_opts.actuator_dt > 0) std::cout << 1.0 / p_sim_opts.actuator_dt << " Hz\n";
        else std::cout << "Inf\n";

        std::cout << std::defaultfloat << std::setprecision(4)
                  << "[Simulator]: Initial state  : [" << m_state.transpose() << "]\n"
                  << "[Simulator]: Initial control: [" << m_control.transpose() << "]\n"
                  << "[Simulator]: Initial params : [" << m_params.transpose() << "]\n"
                  << "[Simulator]: Control bounds : [" << p_control_lb.transpose() << "] < u < ["
                  << p_control_ub.transpose() << "]\n";
    }
    bool simulate_once_impl() override
    {
        update_control();

        // State trap: Freeze state such to observe reaction of non-sim algorithms
        if (m_state_trap) { return true; }

        /* Run simulation engine */
        m_dyn_out = m_sim_engine->get_dyn_out(m_control);

        m_sim_engine->simulate_one_step(m_state, m_control);
        m_state = m_sim_engine->get_state();
        m_output = m_sim_engine->get_output(); // Could apply a sensor model to m_output here

        //std::cout << "m_state: " << m_state.transpose() << ", m_out: " << m_output.transpose() << ", m_dynout: " << m_dyn_out.transpose() << "\n";
        // Could use an additional measured_state variable here equal to [sensor model](m_state).
//        m_output.segment<3>(0) = m_sensor_pose2d.get_noisy(m_output.segment<3>(0)); // Requires angle wrapping (publish_pose2d_raw)
        m_output.segment<3>(3) = m_sensor_vel.get_noisy(m_output.segment<3>(3));

        return true;
    }

    Eigen::VectorXd m_control_in; // Buffer for incoming control, need to shadow set_control and write to control_in
    unsigned m_control_id_in{0};
    Eigen::VectorXd p_control_lb, p_control_ub; // Physical control limits

    /* Actuator update */
    double m_sim_time_actuator_update{0};
    unsigned m_control_id{0};
    void update_control()
    {
        const double sim_time_diff = m_sim_time - m_sim_time_actuator_update;

        if (sim_time_diff >= p_sim_opts.actuator_dt) // actuator_dt is already corrected by simulation speed factor
        {
            m_sim_time_actuator_update = m_sim_time;

            /* Only update servo commands at some rate (e.g. 50 Hz) */
            m_control = p_control_lb.cwiseMax(m_control_in.cwiseMin(p_control_ub));// Clip control to physical limits
            m_control_id = m_control_id_in;
            if (m_VERBOSE >= PrintLevel2)
                std::cout << "update control: " << m_control_in.transpose() << "->" << m_control.transpose() << "\n";
        }
    }

    /* Outputs */
    Eigen::Vector3d m_specNongravForce{0, 0, 0};
    sensor_models::SimpleSensor3D m_sensor_pose2d{0.0008e-6, 0.0005e-6, 0.1021e-6};
    sensor_models::SimpleSensor3D m_sensor_vel{8e-3, 8e-3, 1e-3};

    /* Misc */
    bool m_state_trap{false};
};

} // namespace driving
} // namespace simulation

#endif //SRC_DRIVINGSIMULATOR_HPP
