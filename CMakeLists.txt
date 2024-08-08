cmake_minimum_required(VERSION 2.8.3)
project(im1r_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg FILES IM1R_EXTRA.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package()

###########
## Build ##
###########

# include_directories(
# # include
  # ${catkin_INCLUDE_DIRS}
# )