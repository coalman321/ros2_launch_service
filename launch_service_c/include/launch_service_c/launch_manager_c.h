#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/action/bringup_start.hpp>
#include <launch_msgs/srv/bringup_status.hpp>
#include <tinyxml2.h>

namespace launch_manager {
    class LaunchManager: public rclcpp::Node {
        private:
            // Map the PIDs of the launch files to the tuple of their required subscriptions and their status
            std::map<int, std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr, bool>>> bringup_listeners;
            
            rclcpp::Service<launch_msgs::srv::BringupStatus>::SharedPtr bringup_status;
            rclcpp_action::Server<launch_msgs::action::BringupEnd>::SharedPtr bringup_end;
            rclcpp_action::Server<launch_msgs::action::BringupStart>::SharedPtr bringup_start;
            //Some form of XML document that contains relivent 
        public:
            LaunchManager();
    };
}