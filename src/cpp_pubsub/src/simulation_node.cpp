#include <chrono>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/joint_state.hpp"         // Message for publisher
#include "tutorial_interfaces/msg/joint_torques_command.hpp" // Message for subscriber

using namespace std::chrono_literals;

class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode()
  : Node("simulation_node"), count_(0)
  {
    // Publisher initialization
    publisher_ = this->create_publisher<tutorial_interfaces::msg::JointState>("joint_states", 10);
    
    // Timer for publishing messages
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimulationNode::publish_joint_state, this));
    
    // Subscriber initialization
    subscription_ = this->create_subscription<tutorial_interfaces::msg::JointTorquesCommand>(
      "joint_torques", 10, std::bind(&SimulationNode::receive_joint_torques, this, std::placeholders::_1));
  }

private:
  void publish_joint_state()
  {
    auto message = tutorial_interfaces::msg::JointState();
    
    // Populate the JointState message with dummy data
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.positions = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    message.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    message.torques = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f};
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing JointState: positions = " <<
      message.positions[0] << ", velocities = " << message.velocities[0] << ", torques = " << message.torques[0]);
    
    publisher_->publish(message);
  }

  void receive_joint_torques(const tutorial_interfaces::msg::JointTorquesCommand & msg) const
  {
    // Log the received torques data
    RCLCPP_INFO(this->get_logger(), "Received JointTorquesCommand:");
    
    for (size_t i = 0; i < 6; ++i) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Torque[" << i << "]: " << msg.torques[i]);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<tutorial_interfaces::msg::JointTorquesCommand>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
