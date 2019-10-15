# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(siasun_driver)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp actionlib
    trajectory_msgs actionlib_msgs control_msgs sensor_msgs urdf
    industrial_msgs industrial_robot_client industrial_utils simple_message siasun_msgs )
find_package(Boost REQUIRED COMPONENTS system thread)

#roslint_cpp()
#######################################
## Adding directories and definitions #
#######################################
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
add_definitions(-DROS=1)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs
            control_msgs sensor_msgs simple_message urdf industrial_msgs
            industrial_robot_client siasun_msgs 
    INCLUDE_DIRS include
    LIBRARIES
)


set(MSG_SRC_FILES
  src/simple_message/siasun_motion_ctrl.cpp
  src/simple_message/siasun_motion_ctrl_message.cpp
  src/simple_message/siasun_motion_reply.cpp
  src/simple_message/siasun_motion_reply_message.cpp)


set(CLIENT_SRC_FILES
  src/industrial_robot_client/joint_relay_handler.cpp
  src/industrial_robot_client/joint_feedback_relay_handler.cpp
  src/industrial_robot_client/robot_state_interface.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/industrial_robot_client/joint_trajectory_streamer.cpp
)
###########
## Build ##
###########


#-------------------------------------------------------------
# dx100 uses same byte-ordering as most i386-based PCs

# Simple message library
add_library(siasun_simple_message ${MSG_SRC_FILES})
target_link_libraries(siasun_simple_message
  simple_message)

# Industrial client library
add_library(siasun_industrial_robot_client ${CLIENT_SRC_FILES})
target_link_libraries(siasun_industrial_robot_client
  industrial_robot_client
  industrial_utils)
add_dependencies(siasun_industrial_robot_client ${catkin_EXPORTED_TARGETS})

# Robot state node
add_executable(siasun_robot_state
  src/robot_state_node.cpp)
target_link_libraries(siasun_robot_state
  siasun_simple_message
  siasun_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")

# Motion streaming node
add_executable(siasun_motion_streaming_interface
  src/joint_streaming_node.cpp
  src/joint_trajectory_streamer.cpp
  src/motion_ctrl.cpp)
target_link_libraries(siasun_motion_streaming_interface
  siasun_simple_message
  siasun_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(siasun_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")

#############
## Install ##
#############

# binaries
install(TARGETS
    siasun_robot_state
    siasun_motion_streaming_interface

    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# libraries
install(TARGETS
    siasun_simple_message
    siasun_industrial_robot_client

    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# other files
foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
