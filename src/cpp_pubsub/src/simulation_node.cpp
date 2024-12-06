#include <chrono>
#include <memory>
#include <array>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/joint_state.hpp"         // Message for publisher
#include "tutorial_interfaces/msg/joint_torques_command.hpp" // Message for subscriber

using namespace std::chrono_literals;

float camera_azimuth = 90.0;
float camera_elevation = -20.0;
float camera_distance = 2.0;
float camera_sensitivity = 0.5;

bool mouse_drag = false;
double last_x = 0.0, last_y = 0.0;

void glfw_error_callback(int error, const char *description)
{
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            mouse_drag = true;
        }
        else if (action == GLFW_RELEASE)
        {
            mouse_drag = false;
        }
    }
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (mouse_drag)
    {
        camera_azimuth -= (xpos - last_x) * camera_sensitivity;
        camera_elevation -= (ypos - last_y) * camera_sensitivity;

        if (camera_elevation > 90.0)
            camera_elevation = 90.0;
        if (camera_elevation < -90.0)
            camera_elevation = -90.0;
    }

    last_x = xpos;
    last_y = ypos;
}


class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode()
  : Node("simulation_node"), count_(0)
  {
    if (!glfwInit())
    {
        std::cerr << "Could not initialize GLFW" << std::endl;
    }
    glfwSetErrorCallback(glfw_error_callback);

    window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulator", nullptr, nullptr);
    if (!window_)
    {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, cursor_position_callback);
    glfwSetKeyCallback(window_, keyboard_callback);

    const char *model_path = "/home/a2rlab/Indy7/mujoco-sim/indy-pubsub/src/cpp_pubsub/models/indy7.xml";

    char error[1000];
    std::memset(error, 0, sizeof(error));

    model_ = mj_loadXML(model_path, nullptr, error, sizeof(error));
    std::cout << error << std::endl;
    std::cout << "Loading model from " << model_path << std::endl;

    data_ = mj_makeData(model_);
    std::cout << "Model loaded successfully" << std::endl;


    mjv_defaultScene(&scene_);
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&options_);
    mjr_defaultContext(&context_);

    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);

    camera_.type = mjCAMERA_FREE;
    camera_.distance = camera_distance;
    camera_.azimuth = camera_azimuth;
    camera_.elevation = camera_elevation;

    // Publisher initialization
    publisher_ = this->create_publisher<tutorial_interfaces::msg::JointState>("joint_states_", 10);

    // Timer to publish joint state every 100ms
    joint_state_publish_timer_ = this->create_wall_timer(
      1000ms, std::bind(&SimulationNode::publish_joint_state, this));

    // Subscriber initialization to receive joint torques
    // To publish to subscriber via terminal:
    // ros2 topic pub /joint_torques tutorial_interfaces/msg/JointTorquesCommand "{torques:- [100.0, 2.0, 3.0, 4.0, 5.0, 6.0]}"
    subscription_ = this->create_subscription<tutorial_interfaces::msg::JointTorquesCommand>(
      "joint_torques", 10, std::bind(&SimulationNode::receive_joint_torques, this, std::placeholders::_1));

    // Timer to step the simulation every 10ms
    simulation_step_timer_ = this->create_wall_timer(
      0ms, std::bind(&SimulationNode::simulation_step, this));
  }

  ~SimulationNode()
  {
    // Cleanup resources
    mj_deleteData(data_);
    mj_deleteModel(model_);
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

private:
  void publish_joint_state()
  {
    auto message = tutorial_interfaces::msg::JointState();
    
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    
    for (size_t i = 0; i < 6; ++i) {
      message.positions[i] = data_->qpos[i];
      message.velocities[i] = data_->qvel[i];
      message.torques[i] = data_->act[i];
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing JointState: positions = " <<
      message.positions[0] << ", velocities = " << message.velocities[0] << ", torques = " << message.torques[0]);
    
    publisher_->publish(message);
  }

  void receive_joint_torques(const tutorial_interfaces::msg::JointTorquesCommand & msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received JointTorquesCommand:");
    
    for (size_t i = 0; i < 6; ++i) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Torque[" << i << "]: " << msg.torques[i]);
    }

    for (size_t i = 0; i < 6; ++i) {
      received_torques_[i] = msg.torques[i];
    }
  }

  void simulation_step()
  {
    for (size_t i = 0; i < 6; ++i) {
      data_->ctrl[i] = received_torques_[i];
    }
    
    mj_step(model_, data_);

    camera_.azimuth = camera_azimuth;
    camera_.elevation = camera_elevation;
    camera_.distance = camera_distance;

    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);

    mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(model_, data_, &options_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  rclcpp::TimerBase::SharedPtr joint_state_publish_timer_;
  rclcpp::TimerBase::SharedPtr simulation_step_timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<tutorial_interfaces::msg::JointTorquesCommand>::SharedPtr subscription_;

  mjData *data_;
  mjModel *model_;
  mjvScene scene_;
  mjvCamera camera_;
  mjvOption options_;
  mjrContext context_;
  GLFWwindow *window_;
  size_t count_;

  std::array<double, 6> received_torques_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
