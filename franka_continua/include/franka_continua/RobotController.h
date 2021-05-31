// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <functional>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace franka_continua
{
    class RobotController
    {
    public:
        RobotController(const std::shared_ptr<franka::Model> model_ptr);

        // TODO add control callbacks

    private:
        std::shared_ptr<franka::Model> model_ptr_;
    };
}