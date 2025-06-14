cmake_minimum_required(VERSION 3.5)
project(bottle_detection_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  bottle_detection_ros2/bottle_detection_node.py
  bottle_detection_ros2/bottle_detection_node_async.py
  bottle_detection_ros2/integrated_bottle_detection_node.py
  bottle_detection_ros2/websocket_bridge_node.py
  bottle_detection_ros2/robot_control_node.py
  bottle_detection_ros2/servo_control_node.py
  bottle_detection_ros2/auto_harvest_controller.py
  bottle_detection_ros2/nodes/testing/fruit_image_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install models directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install data directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
  install(DIRECTORY
    data
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()