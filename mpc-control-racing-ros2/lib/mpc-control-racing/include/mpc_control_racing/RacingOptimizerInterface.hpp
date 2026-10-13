#ifndef SRC_RACINGMPCINTERFACE_HPP
#define SRC_RACINGMPCINTERFACE_HPP

#include "mpc_control_common/OptimizerInterface.hpp"
#include "mpc_control_racing/RacingOcpParams.hpp"

namespace mpc_control_racing {

class RacingOptimizerInterface : public mpc_control_common::OptimizerInterface,
                                 public mpc_control_racing::RacingOcpParams
{
public:
    // Space for racing MPC specific members
};

} // namespace racing_mpc

#endif //SRC_RACINGMPCINTERFACE_HPP
