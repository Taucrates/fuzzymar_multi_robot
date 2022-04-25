cmake_minimum_required(VERSION 3.0.2)
project(fuzzymar_multi_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2_ros
  gazebo_msgs
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   global_goal.msg
   task.msg
   taskArray.msg
   action_task.msg
   port.msg
   task_w_ports.msg
   task_w_portsArray.msg
   action_task_w_ports.msg
   taskObjective.msg
   time_task.msg
   robotParameters.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   gazebo_msgs
 )

 generate_dynamic_reconfigure_options(
  cfg/kobukis.cfg
  #...
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES fuzzymar_multi_robot
  CATKIN_DEPENDS roscpp roslaunch rospy std_msgs geometry_msgs message_runtime tf2_ros gazebo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## libraries
add_library(libraries 
            src/stimulus.cpp
            )
target_link_libraries(libraries ${catkin_LIBRARIES})

## nodes
add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})

add_executable(task_ports src/task_ports.cpp)
target_link_libraries(task_ports ${catkin_LIBRARIES})

add_executable(monitor_ports src/monitor_ports.cpp)
target_link_libraries(monitor_ports ${catkin_LIBRARIES})

add_executable(det_robot_operation_ports src/det_robot_operation_ports.cpp)
target_link_libraries(det_robot_operation_ports
                      libraries
                      ${catkin_LIBRARIES})

add_executable(pos_robot_operation_ports src/pos_robot_operation_ports.cpp)
target_link_libraries(pos_robot_operation_ports
                      libraries
                      ${catkin_LIBRARIES})

## deps
add_dependencies(monitor fuzzymar_multi_robot_generate_messages_cpp)
add_dependencies(task_ports fuzzymar_multi_robot_generate_messages_cpp)
add_dependencies(monitor_ports fuzzymar_multi_robot_generate_messages_cpp)
add_dependencies(det_robot_operation_ports fuzzymar_multi_robot_generate_messages_cpp)
add_dependencies(pos_robot_operation_ports fuzzymar_multi_robot_generate_messages_cpp)

## dynamic_reconfigure deps
add_dependencies(pos_robot_operation_ports fuzzymar_multi_robot_gencfg)