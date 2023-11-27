// SPDX-License-Identifier: Apache-2.0
#ifndef AISAAC_MSG_RECEIVER_HPP_
#define AISAAC_MSG_RECEIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aisaac_msgs/msg/aisaac_strategy.hpp"
#include "aisaac_msgs/msg/aisaac_referee.hpp"
#include "aisaac_msgs/msg/aisaac_manual_control.hpp"

namespace aisaac
{

class MsgReceiver : public rclcpp::Node
{
public:
    MsgReceiver(const rclcpp::NodeOptions &options);
protected:
    void test_callback(const std_msgs::msg::String::SharedPtr msg);
    void aisaac_strategy_callback(const aisaac_msgs::msg::AisaacStrategy::SharedPtr msg);
    void aisaac_referee_callback(const aisaac_msgs::msg::AisaacReferee::SharedPtr msg);
    void aisaac_manual_control_callback(const aisaac_msgs::msg::AisaacManualControl::SharedPtr msg);
    void ssl_vision_callback();
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_subscription_;
    rclcpp::Subscription<aisaac_msgs::msg::AisaacStrategy>::SharedPtr aisaac_strategy_subscription_;
    rclcpp::Subscription<aisaac_msgs::msg::AisaacReferee>::SharedPtr aisaac_referee_subscription_;
    rclcpp::Subscription<aisaac_msgs::msg::AisaacManualControl>::SharedPtr aisaac_manual_control_subscription_;
    // rclcpp::Subscription<aisaac_msgs::msg::AisaacStrategy>::SharedPtr ssl_vision_subscription_;
};

}   // namespace aisaac

#endif // AISAAC_MSG_RECEIVER_HPP_