cmake_minimum_required(VERSION 2.8.3)
project(robot_tracker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs message_generation message_filters robotics_hw1 tf dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)
add_service_files(FILES SetPose.srv ResetPose.srv)

add_message_files(FILES CustomMessage.msg)
generate_messages(DEPENDENCIES std_msgs nav_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs nav_msgs geometry_msgs message_filters)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot_tracker src/main.cpp src/SimpleRobot.h)
add_dependencies(robot_tracker custom_messages_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_tracker ${catkin_LIBRARIES})