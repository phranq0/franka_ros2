// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/** 
 * @brief Used to decide the control mode.
 */
namespace franka_continua
{
    enum class EControlMode
    {
        TIME_LIMIT,      ///< Prioritize the time limit
        DESIRED_VELOCITY ///< Prioritize the desired velocity
    };
}