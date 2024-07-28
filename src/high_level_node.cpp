/// @file high_level_node.cpp - Dexhand ROS2 Control Node for Servo Abstraction Layer and Approximate Joint Angle Control
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#include <string>
#include "base_node.hpp"
#include "dexhand_servomgr.hpp"
#include "dexhand_joint_angle_controller.hpp"

#include "dexhandv2_control/msg/servo_targets_table.hpp"
#include "dexhandv2_control/msg/servo_targets_n_table.hpp"
#include "dexhandv2_control/msg/servo_status_table.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

using namespace dexhand_connect;
using namespace std;

class HighLevelControlNode : public DexHandBase {
    public:
        HighLevelControlNode() : DexHandBase("dexhandv2_hlc") {

            // Create parameter to disable joint angle control
            this->declare_parameter<bool>("enable_joint_state_control", true);

            // Create subscribers for servo position messages
            st_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsTable>(
                "dexhandv2/servo_targets", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                std::bind(&HighLevelControlNode::servo_targets_callback, this, std::placeholders::_1));

            st_n_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsNTable>(
                "dexhandv2/servo_targets_n", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                std::bind(&HighLevelControlNode::servo_targets_n_callback, this, std::placeholders::_1));

            // Subscribe to joint_states topic if joint angle control is enabled
            if (this->get_parameter("enable_joint_state_control").as_bool()) {
                RCLCPP_INFO(this->get_logger(), "Joint state control enabled");
                js_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                    "joint_states", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                    std::bind(&HighLevelControlNode::joint_state_callback, this, std::placeholders::_1));
            }

            // Create publisher for servo status messages
            servo_status_publisher = this->create_publisher<dexhandv2_control::msg::ServoStatusTable>(
                "dexhandv2/servo_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
            
            // Create timer for publishing servo status messages on 1 sec interval
            servo_status_timer = this->create_wall_timer(1s, [this]() {
                
                dexhandv2_control::msg::ServoStatusTable msg;
                for (auto& hand : getHands()) {
                    msg.id = hand->getID();
                    auto hlcHand = dynamic_pointer_cast<HLCHandInstance>(hand);

                    // Make sure hand is running
                    if (hlcHand->getServoManager().isReady() == false) {
                        continue;
                    }

                    for (auto& servo : hlcHand->getServoManager().getServos()) {
                        dexhandv2_control::msg::ServoStatus sd;
                        sd.servo_id = servo.second->getID();
                        sd.status = servo.second->getStatus();
                        sd.position = servo.second->getPosition();
                        sd.load = servo.second->getLoad();
                        sd.voltage = servo.second->getVoltage();
                        sd.temp = servo.second->getTemperature();
                        msg.servo_table.push_back(sd);
                    }
                }
                servo_status_publisher->publish(msg);
            });


            // Enumerate all the devices and create hand instances
            this->enumerate_devices([](string deviceID, rclcpp::Node* parent) -> std::shared_ptr<HandInstance> {
                return std::make_shared<HLCHandInstance>(deviceID, parent);
            });

        }

        ~HighLevelControlNode() override {}

    private:

        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // We need to map ROS/URDF joint names to the joint names used by the hand SDK and set the joint
            for (unsigned ji = 0; ji < msg->name.size(); ji++) {
                string name = msg->name[ji];

                bool isRightHand = name.find("R_") != string::npos;

                // Skip DIP and roll joints
                if (name.find("_DIP") != string::npos || name.find("_Roll") != string::npos) {
                    continue;
                }

                // Remove the "R_" or "L_" prefix
                string jointName = name.substr(2);

                // Convert the name to lowercase
                std::transform(jointName.begin(), jointName.end(), jointName.begin(), ::tolower);

                // Look up the joint id in the hand SDK
                std::shared_ptr<HLCHandInstance> firstHand = dynamic_pointer_cast<HLCHandInstance>(getHands()[0]);
                JointAngleController::JointID jointid = firstHand->getJointAngleController().getJointID(jointName);

                if (jointid == JointAngleController::NUM_JOINT_IDS) {
                    RCLCPP_ERROR(this->get_logger(), "Joint %s not found in SDK", jointName.c_str());
                    continue;
                }

                RCLCPP_DEBUG(this->get_logger(), "Mapped ROS Joint %s to SDK Joint %s", name.c_str(), jointName.c_str());
                RCLCPP_DEBUG(this->get_logger(), "Setting joint %s (ID:%d) to position %f", jointName.c_str(), jointid, msg->position[ji]*180.0/M_PI);

                // Set the joint angle
                for (auto& hand : getHands()) {
                    auto hlcHand = dynamic_pointer_cast<HLCHandInstance>(hand);
                    
                    // Only send messages with correct handedness
                    if (hlcHand->getJointAngleController().isRightHand() == isRightHand) {
                        hlcHand->getJointAngleController().setJointAngle(jointid, msg->position[ji]*180.0/M_PI);
                    }
                }

                
            }
        }

        void servo_targets_callback(const dexhandv2_control::msg::ServoTargetsTable::SharedPtr msg) {
            for (auto& hand : getHands()) {
                if (msg->id == hand->getID()) {
                    for (const auto& target : msg->servo_table) {
                        RCLCPP_DEBUG(this->get_logger(), "Setting servo %d to position %d", target.servo_id, target.position);
                        
                        // Cast the hand instance to HLCHandInstance and call the setServoPosition method
                        auto hlcHand = dynamic_pointer_cast<HLCHandInstance>(hand);
                        hlcHand->getServoManager().getServo(target.servo_id)->setTarget(target.position);
                    }
                }
            }
        }

        void servo_targets_n_callback(const dexhandv2_control::msg::ServoTargetsNTable::SharedPtr msg) {
            for (auto& hand : getHands()) {
                if (msg->id == hand->getID()) {
                    for (const auto& target : msg->servo_table) {
                        RCLCPP_DEBUG(this->get_logger(), "Setting servo %d to position %f", target.servo_id, target.position);
                        
                        // Cast the hand instance to HLCHandInstance and call the setServoPosition method
                        auto hlcHand = dynamic_pointer_cast<HLCHandInstance>(hand);
                        hlcHand->getServoManager().getServo(target.servo_id)->setTargetNormalized(target.position);
                    }
                }
            }
        }


        /// @brief Container class for holding instances of DexHand devices discovered by the node. HLC includes a ServoManager and 
        /// (optionally) a JointAngleController object for each instance
        class HLCHandInstance : public HandInstance {

        public:
            const static int SM_TX_RATE = 100; // 100hz
            const static int SM_RX_RATE = 50; // 50hz
            
            HLCHandInstance(string deviceID, rclcpp::Node* parent) : HandInstance(deviceID, parent), servoMgr(getHand()), jac(servoMgr) {
                servoMgr.start(SM_TX_RATE, SM_RX_RATE);
                jac.start();
            }

            ~HLCHandInstance() {
                servoMgr.stop();
            }

            inline ServoManager& getServoManager() { return servoMgr; }
            inline JointAngleController& getJointAngleController() { return jac; }

        private:

            ServoManager servoMgr; 
            JointAngleController jac;
            
    };

    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsTable>::SharedPtr st_subscriber;
    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsNTable>::SharedPtr st_n_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber;

    rclcpp::Publisher<dexhandv2_control::msg::ServoStatusTable>::SharedPtr servo_status_publisher;
    rclcpp::TimerBase::SharedPtr servo_status_timer;
    

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HighLevelControlNode>());
    rclcpp::shutdown();
    return 0;
}