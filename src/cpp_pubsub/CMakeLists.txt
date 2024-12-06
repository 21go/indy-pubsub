cmake_minimum_required(VERSION 3.5)

# Set the project name
project(cpp_pubsub)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories for MuJoCo
set(MUJOCO_PATH ${CMAKE_SOURCE_DIR}/external/mujoco)  # Adjust if necessary

# Include MuJoCo headers
include_directories(${MUJOCO_PATH}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/models)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
# Link MuJoCo libraries
link_directories(${MUJOCO_PATH}/build/lib)

# Enable some compiler warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# MuJoCo dependency (you can add more if needed)
find_package(glfw3 REQUIRED)

# Add source files (adjust as necessary)
set(SOURCE_FILES src/simulation_node.cpp)  # Your ROS 2 main source file

# Add RPATH settings before creating the executable
set(CMAKE_INSTALL_RPATH "${MUJOCO_PATH}/build/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Create the executable
add_executable(simulator ${SOURCE_FILES})

# Link MuJoCo and ROS 2 dependencies
ament_target_dependencies(simulator rclcpp tutorial_interfaces)

# Link MuJoCo and GLFW libraries
target_link_libraries(simulator mujoco glfw)

# If you're building on a specific platform, you may need to link the correct MuJoCo library
if(APPLE)
    target_link_libraries(simulator ${MUJOCO_PATH}/build/lib/libmujoco.dylib)
elseif(UNIX)
    target_link_libraries(simulator ${MUJOCO_PATH}/build/lib/libmujoco.so)
elseif(WIN32)
    target_link_libraries(simulator ${MUJOCO_PATH}/build/lib/mujoco.lib)
endif()

# Optional: Set optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# Install the executable
install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME})

ament_package()