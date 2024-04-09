cmake_minimum_required(VERSION 2.8.3)
project(can_msg)

#######################read files########################




file(GLOB ${PROJECT_NAME}_CAN_MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg msg/*.msg GLOB)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)

add_message_files(
  FILES
  ${${PROJECT_NAME}_CAN_MSGS}
  )

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
