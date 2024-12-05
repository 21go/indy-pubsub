#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::JointState>("joint_state_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    
    message.header.stamp = this->get_clock()->now();  // Set current timestamp
    message.header.frame_id = "base_link";  // Set the frame ID (optional)
    
    // Set the joint positions, velocities, and torques
    message.position = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};  // Example positions
    message.velocity = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};  // Example velocities
    message.effort = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};  // Example torques/efforts
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing JointState: " 
                        << "Positions: " << message.position
                        << ", Velocities: " << message.velocity
                        << ", Efforts: " << message.effort);

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;  // Publisher for JointState message
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
