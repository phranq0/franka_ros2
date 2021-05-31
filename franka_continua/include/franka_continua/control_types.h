// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

namespace franka_continua
{
    /** 
    * @brief Used to decide the control mode.
    */
    enum class EControlMode
    {
        TIME_LIMIT,    ///< Satisfies the time limit first
        VELOCITY_PRIOR ///< Prioritizes the desired velocity
    };

    /**
     * @brief Used to store robot states.
     * 
     */
    struct SRobotState
    {
        std::array<double, 7> q;
        std::array<double, 16> O_T_EE;
        std::array<double, 6> O_F_ext_hat_K;
        std::array<double, 7> tau_ext_hat_filtered;
    };
}