cmake_minimum_required(VERSION 3.5)
project(bottle_detection_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# declare the message files
set(msg_files
  "msg/BottleDetection.msg"
  "msg/RobotCommand.msg"
  "msg/RobotStatus.msg"
  "msg/HarvestCommand.msg"
  "msg/ServoCommand.msg"
  "msg/ServoStatus.msg"
)

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES 
    std_msgs 
    sensor_msgs 
    geometry_msgs
)

# export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()