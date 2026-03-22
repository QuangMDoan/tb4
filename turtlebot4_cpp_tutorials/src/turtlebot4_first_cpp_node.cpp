#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlebot4_msgs/msg/user_button.hpp"

class TurtleBot4FirstNode : public rclcpp::Node
{
public:
  TurtleBot4FirstNode() : Node("turtlebot4_first_cpp_node")
  {
    // Subscribe to the /hmi/buttons topic (TurtleBot4 Standard HMI buttons)
    hmi_buttons_subscriber_ = this->create_subscription<turtlebot4_msgs::msg::UserButton>(
      "/hmi/buttons",
      rclcpp::SensorDataQoS(),
      [this](const turtlebot4_msgs::msg::UserButton::SharedPtr msg) {
        hmi_buttons_callback(msg);
      });
  }

private:
  // HMI buttons subscription callback
  void hmi_buttons_callback(
    const turtlebot4_msgs::msg::UserButton::SharedPtr buttons_msg)
  {
    // Button 1 is pressed (index 0)
    if (buttons_msg->button[0]) {
      RCLCPP_INFO(this->get_logger(), "Button 1 Pressed!");
    }
  }

  // HMI Button Subscriber
  rclcpp::Subscription<turtlebot4_msgs::msg::UserButton>::SharedPtr hmi_buttons_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4FirstNode>());
  rclcpp::shutdown();
  return 0;
}
