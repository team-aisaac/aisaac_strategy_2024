#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aisaac_msgs/msg/aisaac_strategy.hpp"
#include "aisaac_msgs/msg/aisaac_referee.hpp"
#include "aisaac_msgs/msg/aisaac_manual_control.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      aisaac_strategy_publisher_ = this->create_publisher<aisaac_msgs::msg::AisaacStrategy>("aisaac_strategy", 10);
      aisaac_referee_publisher_ = this->create_publisher<aisaac_msgs::msg::AisaacReferee>("aisaac_referee", 10);
      aisaac_manual_control_publisher_ = this->create_publisher<aisaac_msgs::msg::AisaacManualControl>("aisaac_manual_control", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      // Strategy
      auto strategy_ = aisaac_msgs::msg::AisaacStrategy();
      strategy_.robot_id = 1;
      RCLCPP_INFO(this->get_logger(), "Publish strategy: %d", strategy_.robot_id);
      aisaac_strategy_publisher_->publish(strategy_);
      // Referee
      auto referee_ = aisaac_msgs::msg::AisaacReferee();
      referee_.halt_flag = count_ %2 == 0 ? true : false;
      RCLCPP_INFO(this->get_logger(), "Publish referee: %s", referee_.halt_flag ? "HALT" : "Not HALT");
      aisaac_referee_publisher_->publish(referee_);
      // Manual control
      auto manual_control_ = aisaac_msgs::msg::AisaacManualControl();
      manual_control_.robot_id = 2;
      RCLCPP_INFO(this->get_logger(), "Publish manual: %d", manual_control_.robot_id);
      aisaac_manual_control_publisher_->publish(manual_control_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<aisaac_msgs::msg::AisaacStrategy>::SharedPtr aisaac_strategy_publisher_;
    rclcpp::Publisher<aisaac_msgs::msg::AisaacReferee>::SharedPtr aisaac_referee_publisher_;
    rclcpp::Publisher<aisaac_msgs::msg::AisaacManualControl>::SharedPtr aisaac_manual_control_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}