// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#include "franka_continua/RobotController.h"

namespace franka_continua
{
    RobotController::RobotController(std::shared_ptr<franka::Model> model_ptr) 
    {
        model_ptr_ = model_ptr;
    }

}