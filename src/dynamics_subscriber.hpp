/// @file dynamics_subscriber.hpp - Subscriber for DexHand servo dynamics messages
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dexhand_connect.hpp"
#include "dexhandv2_control/msg/servo_dynamics_table.hpp"



class DynamicsSubscriber : public dexhand_connect::IDexhandMessageSubscriber<dexhand_connect::ServoDynamicsMessage> {
    public:
        DynamicsSubscriber(std::string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()){
            // Define QoS policy - best effort, keep last message, volatile
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();

            sd_publisher = parent->create_publisher<dexhandv2_control::msg::ServoDynamicsTable>("dexhandv2/servo_dynamics", qos);
        }
        ~DynamicsSubscriber() = default;

        void messageReceived(const dexhand_connect::ServoDynamicsMessage& message) override {
            RCLCPP_DEBUG(logger, "Dynamics message received for device: %s", deviceID.c_str());
            RCLCPP_DEBUG(logger, "Num servos: %ld", message.getNumServos());
            RCLCPP_DEBUG(logger, "ID\tStatus\tPos\tSpd\tLoad");
            RCLCPP_DEBUG(logger, "------------------------------------");
            

            dexhandv2_control::msg::ServoDynamicsTable sd_msg;
            sd_msg.id = deviceID;

            for (const auto& status : message.getServoStatus()){
                RCLCPP_DEBUG(logger, "%d\t%d\t%d\t%d\t%d", (int)status.first, status.second.getStatus(), status.second.getPosition(), status.second.getSpeed(), status.second.getLoad());
                
                dexhandv2_control::msg::ServoDynamics sd;
                sd.servo_id = status.first;
                sd.status = status.second.getStatus();
                sd.position = status.second.getPosition();
                sd.speed = status.second.getSpeed();
                sd.load = status.second.getLoad();
                sd_msg.servo_table.push_back(sd);
            }
            
            sd_publisher->publish(sd_msg);
        }
    private:
        std::string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::ServoDynamicsTable>::SharedPtr sd_publisher;
};
