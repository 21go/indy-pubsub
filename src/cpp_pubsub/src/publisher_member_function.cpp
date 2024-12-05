#include <chrono>
#include <memory>
#include <array>  // Include for array type

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/joint_state.hpp"  // Change to use JointState

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::JointState>("topic", 10);  // Change to JointState
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::JointState();  // Change to JointState

    // Hardcoded dummy data for the JointState message
    message.header.stamp = this->get_clock()->now();  // Set timestamp
    message.header.frame_id = "base_link";            // You can modify this as needed

    // Fill positions, velocities, and torques with dummy data
    message.positions = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    message.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    message.torques = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f};

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing JointState: positions = " <<
      message.positions[0] << ", velocities = " << message.velocities[0] << ", torques = " << message.torques[0]);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::JointState>::SharedPtr publisher_;  // Change to JointState
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}