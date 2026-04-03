#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

class JoyButtonBridge : public rclcpp::Node
{
public:
  JoyButtonBridge()
  : Node("joy_button_bridge")
  {
    // Subscriber to joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&JoyButtonBridge::joy_callback, this, std::placeholders::_1));

    // Publisher for auxiliary commands
    aux_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/esp32_aux_cmd", 10);

    // Initialize button states (assume 10 buttons max)
    previous_button_states_.resize(10, 0);

    // Define button mappings
    // Button index -> command string
    button_commands_[0] = "center_pan_tilt,0";  // Button A
    button_commands_[1] = "toggle_led,1";       // Button B
    button_commands_[2] = "estop,1";            // Button X
    button_commands_[3] = "beep,1";             // Button Y

    RCLCPP_INFO(this->get_logger(), "Joy button bridge node started");
    RCLCPP_INFO(this->get_logger(), "Button mappings:");
    RCLCPP_INFO(this->get_logger(), "  Button 0 (A) -> center_pan_tilt,0");
    RCLCPP_INFO(this->get_logger(), "  Button 1 (B) -> toggle_led,1");
    RCLCPP_INFO(this->get_logger(), "  Button 2 (X) -> estop,1");
    RCLCPP_INFO(this->get_logger(), "  Button 3 (Y) -> beep,1");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ensure we have enough space for all buttons
    if (msg->buttons.size() > previous_button_states_.size())
    {
      previous_button_states_.resize(msg->buttons.size(), 0);
    }

    // Check each button for rising edge (0 -> 1 transition)
    for (size_t i = 0; i < msg->buttons.size(); ++i)
    {
      int current_state = msg->buttons[i];
      int previous_state = previous_button_states_[i];

      // Detect rising edge (button press)
      if (current_state == 1 && previous_state == 0)
      {
        // Check if we have a mapping for this button
        auto it = button_commands_.find(i);
        if (it != button_commands_.end())
        {
          // Publish the auxiliary command
          auto aux_msg = std_msgs::msg::String();
          aux_msg.data = it->second;
          aux_cmd_pub_->publish(aux_msg);

          RCLCPP_INFO(this->get_logger(), "Button %zu pressed, sending: %s",
                      i, aux_msg.data.c_str());
        }
      }

      // Update previous state
      previous_button_states_[i] = current_state;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aux_cmd_pub_;

  std::vector<int> previous_button_states_;
  std::map<size_t, std::string> button_commands_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyButtonBridge>());
  rclcpp::shutdown();
  return 0;
}
