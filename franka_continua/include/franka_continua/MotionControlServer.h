// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot.h>

#include "franka_continua/control_types.h"
#include "franka_continua/RobotController.h"
#include "franka_interfaces/msg/franka_state.hpp"
#include "franka_interfaces/srv/franka_control.hpp"

namespace franka_continua
{
    /**
     * @brief Receives motion commands, executes the controller and publishes robot states.
     * 
     */
    class MotionControlServer : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit MotionControlServer(rclcpp::NodeOptions options, const std::string &robot_ip);

    protected:
        /**
         * @brief Configures the robot controller, control service and state publisher.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         * 
         * `on_configure` callback is being called when the node enters "configuring" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

        /**
         * @brief Sets the default robot behaviours and starts the robot controller thread.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         * 
         * `on_activate` callback is being called when the node enters the "activating" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

        /**
         * @brief Stops the franka control loop and deactivates the state publisher and control service.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
         * 
         * `on_deactivate` callback is being called when the lifecycle node enters the "deactivating" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

        /**
         * @brief Clear all configurations and robot handler.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         * 
         * `on_cleanup` callback is being called when the lifecycle node enters the "cleaningup" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

        /**
         * @brief Stops the robot controller thread.
         * 
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         * 
         * `on_shutdown` callback is being called when the lifecycle node enters the "shuttingdown" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    private:
        /**
         * @brief Control service callback function that receives control request and modifies the robot controller parameters.
         * 
         * @param request 
         * @param response
         * 
         * TODO put reference of intro to different types of control commands here
         */
        void ControlServiceCallback(const std::shared_ptr<franka_interfaces::srv::FrankaControl::Request> request, std::shared_ptr<franka_interfaces::srv::FrankaControl::Response> response);

        /**
         * @brief Publishes robot states at a fixed rate.
         * 
         * The publisher only starts to publish `FrankaState` msg after it is activated.
         * 
         */
        void PublishState();

        rclcpp::Service<franka_interfaces::srv::FrankaControl>::SharedPtr control_service_;                    ///< Control service handler
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<franka_interfaces::msg::FrankaState>> state_pub_; ///< State publisher handler
        std::shared_ptr<rclcpp::TimerBase> pub_timer_;                                                         ///< A timer periodically triggering the state publisher

        std::unique_ptr<franka::Robot> robot_ptr_;                      ///< Franka robot handler
        std::unique_ptr<franka_continua::SRobotState> robot_state_ptr_; ///< Synchronizes useful robot states for publisher
        std::unique_ptr<franka_continua::RobotController> rcp_;         ///< Robot controller pointer
        std::thread controller_thread_;                                 ///< Continuous robot control loop

        std::mutex mutex_;                                    ///< Protects motion parameters
        double time_limit_;                                   ///< Maximum motion time
        bool motion_finished_;                                ///< Switch of the control loop
        std::string error_;                                   ///< Stores the last error msg
        franka_continua::EControlMode control_mode_;          ///< Robot controller mode
        std::array<std::vector<double>, 6> desired_pose_;     ///< Cartesian waypoints
        std::array<std::vector<double>, 6> desired_velocity_; ///< Desired velocities
    };
}