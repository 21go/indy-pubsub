#include <functional>
#include <memory>
#include <array>  // Include for fixed-size array handling

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/joint_torques_command.hpp"  // Include the correct message type

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::JointTorquesCommand>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::JointTorquesCommand & msg) const
  {
    // Log the received torques data
    RCLCPP_INFO(this->get_logger(), "I heard torques:");

    // Loop through the fixed-size array of torques and log each value
    for (size_t i = 0; i < 6; ++i) {  // Explicitly use size 6
      RCLCPP_INFO_STREAM(this->get_logger(), "Torque[" << i << "]: " << msg.torques[i]);
    }
  }

  rclcpp::Subscription<tutorial_interfaces::msg::JointTorquesCommand>::SharedPtr subscription_;  // Subscription type updated
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
