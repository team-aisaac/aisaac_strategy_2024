//
// Created by ryuzo on 2023/02/16.
//
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include "rclcpp_components/register_node_macro.hpp"
#include "manual_node/manual_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nlohmann/json.hpp"
#include "utilities/can_utils.hpp"
#include "tcp_interface/srv/tcp_socket_i_ctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using json = nlohmann::json;

namespace manual_node {

    ManualNode::ManualNode(const rclcpp::NodeOptions &options) : Node("manual_node", options) {
        using namespace std::chrono_literals;
        tcp8011_flag = false;

        declare_parameter("interval_ms", 10);
        interval_ms = this->get_parameter("interval_ms").as_int();
        _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", _qos);
        client_ = this->create_client<tcp_interface::srv::TcpSocketICtrl>("/tcp_interface/tcp_register");
        manual_instruction["x"] = 0;
        manual_instruction["y"] = 0;
        manual_instruction["rad"] = 0;
        while (!client_->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Manual Node Client interrupted while waiting for TCP service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for TCP service");
        }

        auto request_tcp_8011 = std::make_shared<tcp_interface::srv::TcpSocketICtrl::Request>();
        request_tcp_8011->port = 8011;
        request_tcp_8011->timeout = -1;
        auto future_response_tcp_8011 = client_->async_send_request(request_tcp_8011, std::bind(&ManualNode::_callback_response_tcp8011,
                                                                                                this, std::placeholders::_1));
        _subscription_tcp_8011 = this->create_subscription<std_msgs::msg::String>("/tcp_8011",
                                                                                           _qos,
                                                                                           std::bind(&ManualNode::_subscriber_callback_tcp_8011, this, std::placeholders::_1)
                                                                                           );
        _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                [this] { _publisher_callback(); }
                );
    }

    void ManualNode::_subscriber_callback_tcp_8011(std_msgs::msg::String msg) {
        std::string content = msg.data;
        //RCLCPP_INFO(this->get_logger(), "content:%s", content.c_str());
        try
        {
            manual_instruction = json::parse(content);
        }
        catch (std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "ERROR AT ManualNode::_subscriber_callback_tcp_8011, IGNORING");
        }
    }

    void ManualNode::_publisher_callback() {
        if (!this->tcp8011_flag){
            return;
        }
        float x_val = manual_instruction["x"];
        float y_val = manual_instruction["y"];
        float rad = manual_instruction["rad"];
        RCLCPP_INFO(this->get_logger(), "x:%f, y:%f, rad:%f", x_val, y_val, rad);
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = x_val;
        msg->linear.y = y_val;
        msg->angular.z = rad;
        _publisher_cmd_vel->publish(*msg);
    }

    void ManualNode::_callback_response_tcp8011(rclcpp::Client<tcp_interface::srv::TcpSocketICtrl>::SharedFuture future) {
        if (future.get()->ack){
            this->tcp8011_flag = true;
        } else{
            RCLCPP_ERROR(this->get_logger(), "TCP service could not earn 8011 port");
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(manual_node::ManualNode)
