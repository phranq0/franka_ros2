// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot.h>

#include "franka_interfaces/msg/franka_state.hpp"
#include "franka_interfaces/srv/franka_control.hpp"

namespace franka_continua
{
    /**
     * @brief Receives motion commands, executes the controller and publishes robot states.
     */
    class MotionControlServer : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit MotionControlServer(rclcpp::NodeOptions options, const std::string& robot_ip);

        /**
         * @brief Configures the robot handler, franka control server and robot state publisher.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

    private:
        void ControlServiceCallback(const std::shared_ptr<franka_interfaces::srv::FrankaControl::Request> request, std::shared_ptr<franka_interfaces::srv::FrankaControl::Response> response);

        rclcpp::Service<franka_interfaces::srv::FrankaControl>::SharedPtr control_service_;                    ///< Motion control service
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<franka_interfaces::msg::FrankaState>> state_pub_; ///< Robot state publisher

        std::unique_ptr<franka::Robot> robot_ptr_;
    };
}