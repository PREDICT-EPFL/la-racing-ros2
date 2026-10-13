#ifndef SRC_AMZMINITYPES_HPP
#define SRC_AMZMINITYPES_HPP

#include "main_control_common/ControlTypes.hpp"

namespace amzmini_types {

/* Manual */
template<typename T  = double>
struct Manual_s
{
    T steering{0};      // RC roll stick position, range: -1 ... 1, positive: right
    T acceleration{0};  // RC pitch stick position,  range: -1 ... 1, positive: max forward
    T param_right_left{0};
    T param_up_down{0};
    bool activation_button{false};

    Manual_s() = default;

    Manual_s(const T &steering, const T &acceleration, bool activation_button) :
            steering(steering), acceleration(acceleration), activation_button(activation_button) {}
};
using Manual = Manual_s<double>;

/* AmzminiCommand - commands to be passed through by the Flight Controller Boards to the servos */
template<typename T = double>
struct AmzminiCommand_s
{
    T steering{0};   // Steering
    T throttle{0};   // Positive: Thrust angle deflection around pitch axis
    T brake{0};      // Positive: +thrust

    AmzminiCommand_s() = default;

    AmzminiCommand_s(const T &steering, const T &throttle, const T &brake) :
            steering(steering), throttle(throttle), brake(brake) {}
};
using AmzminiCommand = AmzminiCommand_s<double>;

} // namespace amzmini_types

#endif //SRC_AMZMINITYPES_HPP
