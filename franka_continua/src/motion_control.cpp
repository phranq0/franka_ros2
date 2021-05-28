// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "franka_continua/MotionControlServer.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto control_server_handler = std::make_shared<franka_continua::MotionControlServer>(options, argv[1]);

    executor.add_node(control_server_handler->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}