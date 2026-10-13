#ifndef SRC_SHIFT_WARM_START_HEADING_CASADI_HPP
#define SRC_SHIFT_WARM_START_HEADING_CASADI_HPP

#include "casadi/casadi.hpp"

namespace mpc_control_racing {
namespace casadi_ocp {

static bool shift_warm_heading(double state_heading, casadi::DM &Heading_warm)
{
    const bool print_debug{false};
    if (print_debug) std::cout << "heading: " << state_heading << ", warm: " << Heading_warm.nonzeros() << "\n";

    bool modify{false};
    for (unsigned i = 0; i < 10; i++)
    {
        double heading_err = state_heading - Heading_warm(0).nonzeros()[0];
        int warm_correction{0};
        if (heading_err < -M_PI) { warm_correction = -1; }
        else if (heading_err > M_PI) { warm_correction = +1; }

        if (warm_correction != 0)
        {
            Heading_warm += warm_correction * 2.0 * M_PI;
            modify = true;
            std::cout << "Warm-start heading += " << warm_correction;
            if (print_debug) std::cout << " => " << Heading_warm << "\n";
            std::cout << "\n";
        }
        else { break; }
    }
    return modify;
}

}
}

#endif //SRC_SHIFT_WARM_START_HEADING_CASADI_HPP
