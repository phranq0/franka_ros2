// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#include <chrono>

#include <franka/model.h>

#include "franka_continua/control_helper.h"
#include "franka_continua/MotionControlServer.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::State;

namespace franka_continua
{
    MotionControlServer::MotionControlServer(rclcpp::NodeOptions options, const std::string &robot_ip) : rclcpp_lifecycle::LifecycleNode("motion_control_server", options)
    {
        robot_ptr_ = std::make_unique<franka::Robot>(robot_ip, GetRealtimeConfig());

        RCLCPP_INFO(get_logger(), "Connected to robot @ %s", robot_ip);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        robot_state_ptr_ = std::make_unique<franka_continua::SRobotState>();
        control_mode_ = EControlMode::VELOCITY_PRIOR;
        time_limit_ = -1;

        auto mp = std::make_shared<franka::Model>(robot_ptr_->loadModel());
        rcp_ = std::make_unique<franka_continua::RobotController>(mp);

        control_service_ = create_service<franka_interfaces::srv::FrankaControl>("/franka_control", std::bind(&MotionControlServer::ControlServiceCallback, this, _1, _2));
        state_pub_ = create_publisher<franka_interfaces::msg::FrankaState>("franka_state_publisher", 100);
        pub_timer_ = create_wall_timer(1ms, std::bind(&MotionControlServer::PublishState, this));

        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        if (error_.length() > 0)
        {
            try
            {
                robot_ptr_->automaticErrorRecovery();
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Automatic recovery failed.");

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
        }

        motion_finished_ = false;
        error_ = "";

        // TODO start controller loop
        // robot_ptr_->control();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        state_pub_->on_deactivate();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        pub_timer_.reset();
        state_pub_.reset();

        robot_ptr_.reset(nullptr);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotionControlServer::on_shutdown(const rclcpp_lifecycle::State &)
    {
        motion_finished_ = true;

        if (controller_thread_.joinable())
        {
            controller_thread_.join();
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void MotionControlServer::ControlServiceCallback(const std::shared_ptr<franka_interfaces::srv::FrankaControl::Request> request, std::shared_ptr<franka_interfaces::srv::FrankaControl::Response> response)
    {
        if (get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
        {
            const std::lock_guard<std::mutex> lock(mutex_);

            // TODO set desired_pose and desired_velocity

            if ((time_limit_ = request->time_limit) > 0)
            {
                control_mode_ = EControlMode::TIME_LIMIT;
            }

            response->success = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Motion control server is not activated. Control service is not launched.");
        }
    }

    void MotionControlServer::PublishState()
    {
        auto msg = std::make_unique<franka_interfaces::msg::FrankaState>();
        msg->header.frame_id = "base";
        msg->header.stamp = get_clock()->now();

        msg->q = robot_state_ptr_->q;
        msg->o_t_ee = robot_state_ptr_->O_T_EE;
        msg->o_f_ext_hat_k = robot_state_ptr_->O_F_ext_hat_K;
        msg->tau_ext_hat_filtered = robot_state_ptr_->tau_ext_hat_filtered;

        if (!state_pub_->is_activated())
        {
            RCLCPP_INFO(get_logger(), "Motion control server is currently inactive. Franka states are not published.");
        }

        state_pub_->publish(std::move(msg));
    }

}