/// @file high_level_node.cpp - Dexhand ROS2 Control Node for Servo Abstraction Layer and Approximate Joint Angle Control
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#include <string>
#include "base_node.hpp"
#include "dexhand_servomgr.hpp"

#include "dexhandv2_control/msg/servo_targets_table.hpp"
#include "dexhandv2_control/msg/servo_targets_n_table.hpp"

using namespace dexhand_connect;
using namespace std;

class HighLevelControlNode : public DexHandBase {
    public:
        HighLevelControlNode() : DexHandBase("dexhandv2_hlc") {

            // Create subscribers for servo position messages
            st_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsTable>(
                "dexhandv2/servo_targets", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                std::bind(&HighLevelControlNode::servo_targets_callback, this, std::placeholders::_1));

            st_n_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsNTable>(
                "dexhandv2/servo_targets_n", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                std::bind(&HighLevelControlNode::servo_targets_n_callback, this, std::placeholders::_1));


            // Enumerate all the devices and create hand instances
            this->enumerate_devices([](string deviceID, rclcpp::Node* parent) -> std::shared_ptr<HandInstance> {
                return std::make_shared<HLCHandInstance>(deviceID, parent);
            });

        }

        ~HighLevelControlNode() override {}

    private:

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
            
            HLCHandInstance(string deviceID, rclcpp::Node* parent) : HandInstance(deviceID, parent), servoMgr(getHand()) {
                servoMgr.start(SM_TX_RATE, SM_RX_RATE);
            }

            ~HLCHandInstance() {
                servoMgr.stop();
            }

            inline ServoManager& getServoManager() { return servoMgr; }

        private:

            ServoManager servoMgr; 
            
    };

    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsTable>::SharedPtr st_subscriber;
    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsNTable>::SharedPtr st_n_subscriber;


};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HighLevelControlNode>());
    rclcpp::shutdown();
    return 0;
}