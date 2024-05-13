#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "aisaac_msgs/msg/aisaac_strategy.hpp"

class AisaacRaspiRelay : public rclcpp::Node
{
public:
    AisaacRaspiRelay() : Node("aisaac_raspi_relay")
    {}
private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AisaacRaspiRelay>());
    rclcpp::shutdown();
    return 0;
}