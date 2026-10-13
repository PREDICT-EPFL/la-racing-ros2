#ifndef SRC_ACCELERATEDPATHPARAM_HPP
#define SRC_ACCELERATEDPATHPARAM_HPP

#include <Eigen/Core>

namespace mpc_control_racing{

class AcceleratedPathparam
{
public:
    static constexpr unsigned nx = 2;
    static constexpr unsigned nu = 1;
    static constexpr unsigned np = 0;
    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

    template<typename T>

    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        (void) p;

//        T pathparam_pos = x(0);
        T pathparam_vel = x(1);
        T pathparam_acc = u(0);

        /** ============================================================================================================ **/
        /** Start of model **/

        T pathparam_pos_dot = pathparam_vel;
        T pathparam_vel_dot = pathparam_acc;

        /** End of model **/
        /** ============================================================================================================ **/

        state_t<T> state_dot;
        state_dot << pathparam_pos_dot, pathparam_vel_dot;
        return state_dot;
    }
};

}

#endif //SRC_ACCELERATEDPATHPARAM_HPP
