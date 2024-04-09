cmake_minimum_required(VERSION 2.8.3)
project(trans_probal_score)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  

)

add_message_files(
  FILES
   trans_probal_score.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)
