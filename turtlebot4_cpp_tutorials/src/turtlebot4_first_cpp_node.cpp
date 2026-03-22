#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlebot4_msgs/msg/user_button.hpp"
#include "turtlebot4_msgs/msg/user_led.hpp"

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


    // Create a publisher for the /hmi/led topic
    led_publisher_ = this->create_publisher<turtlebot4_msgs::msg::UserLed>(
      "/hmi/led",
      rclcpp::SensorDataQoS());      
  }

private:

  // HMI buttons subscription callback
  void hmi_buttons_callback(
    const turtlebot4_msgs::msg::UserButton::SharedPtr buttons_msg)
  {
    // Button 1 is pressed (index 0)
    if (buttons_msg->button[0]) {
      RCLCPP_INFO(this->get_logger(), "Button 1 Pressed!");
      button_1_function();
    }
  }

  // Perform a function when Button 1 is pressed.
  void button_1_function()
  {
    // Create ROS 2 messages for each LED
    auto led1_msg = turtlebot4_msgs::msg::UserLed();
    auto led2_msg = turtlebot4_msgs::msg::UserLed();

    led1_msg.led = turtlebot4_msgs::msg::UserLed::USER_LED_1;
    led2_msg.led = turtlebot4_msgs::msg::UserLed::USER_LED_2;

    // Lights are currently off
    if (!lights_on_) {
      // LED 1 - Red, solid
      led1_msg.color = turtlebot4_msgs::msg::UserLed::COLOR_RED;
      led1_msg.blink_period = 0;
      led1_msg.duty_cycle = 1.0;

      // LED 2 - Green, solid
      led2_msg.color = turtlebot4_msgs::msg::UserLed::COLOR_GREEN;
      led2_msg.blink_period = 0;
      led2_msg.duty_cycle = 1.0;
    }
    // Lights are currently on
    else {
      // Turn off both LEDs
      led1_msg.color = turtlebot4_msgs::msg::UserLed::COLOR_OFF;
      led1_msg.blink_period = 0;
      led1_msg.duty_cycle = 0.0;

      led2_msg.color = turtlebot4_msgs::msg::UserLed::COLOR_OFF;
      led2_msg.blink_period = 0;
      led2_msg.duty_cycle = 0.0;
    }
    // Publish the messages
    led_publisher_->publish(led1_msg);
    led_publisher_->publish(led2_msg);
    // Toggle the lights on status
    lights_on_ = !lights_on_;
  }
  
  
  // HMI Button Subscriber
  rclcpp::Subscription<turtlebot4_msgs::msg::UserButton>::SharedPtr hmi_buttons_subscriber_;

  // HMI LED Publisher
  rclcpp::Publisher<turtlebot4_msgs::msg::UserLed>::SharedPtr led_publisher_;

  // Lights on status
  bool lights_on_{};  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4FirstNode>());
  rclcpp::shutdown();
  return 0;
}
