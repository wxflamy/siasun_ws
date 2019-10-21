cmake_minimum_required(VERSION 2.8.3)
project(siasun_msgs)

set(MSG_DEPS
  std_msgs
  industrial_msgs
)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(SRV_FILES
GripperCtrl.srv
)

add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})
