#ifndef SRC_STACKEDSYSTEM_HPP
#define SRC_STACKEDSYSTEM_HPP

#include <Eigen/Core>

namespace mpc_control_racing {

template<typename Sys1, typename Sys2>
class StackedSystem : public Sys1, public Sys2
{
public:
    static constexpr unsigned nx = Sys1::nx + Sys2::nx;
    static constexpr unsigned nu = Sys1::nu + Sys2::nu;
    static constexpr unsigned np = Sys1::np + Sys2::np;

    template<typename T> using state_t = Eigen::Vector<T, nx>;
    template<typename T> using input_t = Eigen::Vector<T, nu>;
    template<typename T> using param_t = Eigen::Vector<T, np>;

    template<typename T>// T is scalar type
    state_t<T> dynamics_impl(const Eigen::Ref<const state_t<T>> &x,
                             const Eigen::Ref<const input_t<T>> &u,
                             const Eigen::Ref<const param_t<T>> &p)
    {
        typename Sys1::template state_t<T> state_1 = x.topRows(Sys1::nx);
        typename Sys1::template input_t<T> input_1 = u.topRows(Sys1::nu);
        typename Sys1::template param_t<T> param_1 = p.topRows(Sys1::np);
        Eigen::Vector<T, Sys1::nx> state_dot_1 = sys1.template dynamics_impl<T>(state_1, input_1, param_1);

        typename Sys2::template state_t<T> state_2 = x.bottomRows(Sys2::nx);
        typename Sys2::template input_t<T> input_2 = u.bottomRows(Sys2::nu);
        typename Sys2::template param_t<T> param_2 = p.bottomRows(Sys2::np);
        Eigen::Vector<T, Sys2::nx> state_dot_2 = sys2.template dynamics_impl<T>(state_2, input_2, param_2);

        state_t<T> state_dot;
        state_dot << state_dot_1.topRows(Sys1::nx), state_dot_2.topRows(Sys2::nx);
        return state_dot;
    }
protected:
    Sys1 sys1{};
    Sys2 sys2{};
};
}
#endif //SRC_STACKEDSYSTEM_HPP
