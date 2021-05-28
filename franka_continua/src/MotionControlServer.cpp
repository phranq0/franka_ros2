// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#include "franka_continua/control_helper.h"
#include "franka_continua/MotionControlServer.h"

namespace franka_continua
{
    MotionControlServer::MotionControlServer(rclcpp::NodeOptions options, const std::string &robot_ip) : rclcpp_lifecycle::LifecycleNode("motion_control_server", options)
    {
        robot_ptr_ = std::make_unique<franka::Robot>(robot_ip, GetRealtimeConfig());
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_configure(const rclcpp_lifecycle::State &)
    {
        control_service_ = create_service<franka_interfaces::srv::FrankaControl>("/franka_control", std::bind(&MotionControlServer::ControlServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
        state_pub_ = create_publisher<franka_interfaces::msg::FrankaState>("franka_state_publisher", 100);

        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void MotionControlServer::ControlServiceCallback(const std::shared_ptr<franka_interfaces::srv::FrankaControl::Request> request, std::shared_ptr<franka_interfaces::srv::FrankaControl::Response> response)
    {
        response->success = true;
    }

}