#include <memory>
// Serial
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aisaac_msgs/msg/aisaac_strategy.hpp"
#include "aisaac_msgs/msg/aisaac_referee.hpp"
#include "aisaac_msgs/msg/aisaac_manual_control.hpp"
using std::placeholders::_1;

class AisaacRaspiRelay : public rclcpp::Node
{
public:
    AisaacRaspiRelay() : Node("aisaac_raspi_relay")
    {
        strategy_ = this->create_subscription<aisaac_msgs::msg::AisaacStrategy>("strategy", 10, std::bind(&AisaacRaspiRelay::strategy_callback, this, _1));
        referee_ = this->create_subscription<aisaac_msgs::msg::AisaacReferee>("referee", 10, std::bind(&AisaacRaspiRelay::referee_callback, this, _1));
        manual_control_ = this->create_subscription<aisaac_msgs::msg::AisaacManualControl>("manual_control", 10, std::bind(&AisaacRaspiRelay::manual_control_callback, this, _1));
    }
private:
    void strategy_callback(const aisaac_msgs::msg::AisaacStrategy::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "RX: '%d'", msg->robot_id);
    }
    void referee_callback(const aisaac_msgs::msg::AisaacReferee::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "RX: '%d'", msg->in_game);
    }
    void manual_control_callback(const aisaac_msgs::msg::AisaacManualControl::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "RX: '%d'", msg->robot_id);
    }
    rclcpp::Subscription<aisaac_msgs::msg::AisaacStrategy>::SharedPtr strategy_;
    rclcpp::Subscription<aisaac_msgs::msg::AisaacReferee>::SharedPtr referee_;
    rclcpp::Subscription<aisaac_msgs::msg::AisaacManualControl>::SharedPtr manual_control_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AisaacRaspiRelay>());
    rclcpp::shutdown();
    return 0;
}