#ifndef SRC_SHIFT_WARM_START_HEADING_HPP
#define SRC_SHIFT_WARM_START_HEADING_HPP

#include <Eigen/Core>

namespace mpc_control_racing {

static bool shift_warm_heading(double state_heading, Eigen::MatrixXd &Heading_warm)
{
    const bool print_debug{false};
    if (print_debug) std::cout << "heading: " << state_heading << ", warm: " << Heading_warm << "\n";

    bool modify{false};
    for (unsigned i = 0; i < 10; i++)
    {
        double headin_err = state_heading - Heading_warm(0);
        int warm_correction{0};
        if (headin_err < -M_PI) { warm_correction = -1; }
        else if (headin_err > M_PI) { warm_correction = +1; }

        if (warm_correction != 0)
        {
            Heading_warm.array() += warm_correction * 2.0 * M_PI;
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

#endif //SRC_SHIFT_WARM_START_HEADING_HPP
