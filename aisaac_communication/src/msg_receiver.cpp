// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "aisaac_communication/msg_receiver.hpp"

namespace aisaac
{
using std::placeholders::_1;

MsgReceiver::MsgReceiver(const rclcpp::NodeOptions &options)
    : Node("msg_receiver", options)
{
    test_subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MsgReceiver::test_callback, this, _1));
    aisaac_strategy_subscription_ = this->create_subscription<aisaac_msgs::msg::AisaacStrategy>("aisaac_strategy", 10, std::bind(&MsgReceiver::aisaac_strategy_callback, this, _1));
    aisaac_referee_subscription_ = this->create_subscription<aisaac_msgs::msg::AisaacReferee>("aisaac_referee", 10, std::bind(&MsgReceiver::aisaac_referee_callback, this, _1));
    aisaac_manual_control_subscription_ = this->create_subscription<aisaac_msgs::msg::AisaacManualControl>("aisaac_manual_control", 10, std::bind(&MsgReceiver::aisaac_manual_control_callback, this, _1));
}

void MsgReceiver::test_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Rx: %s", msg->data.c_str());
}

void MsgReceiver::aisaac_strategy_callback(const aisaac_msgs::msg::AisaacStrategy::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Rx Strategy: RobotId: %d", msg->robot_id);
}

void MsgReceiver::aisaac_referee_callback(const aisaac_msgs::msg::AisaacReferee::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Rx Referee: %s", msg->halt_flag ? "HALT" : "Not HALT");
}

void MsgReceiver::aisaac_manual_control_callback(const aisaac_msgs::msg::AisaacManualControl::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Rx Manual Cmd: RobotId: %d", msg->robot_id);
}

void MsgReceiver::ssl_vision_callback()
{
    RCLCPP_INFO(this->get_logger(), "Rx SSL Vision");
}

}   // namespace aisaac


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<aisaac::MsgReceiver>(options));
    rclcpp::shutdown();

    return 0;
}
