cmake_minimum_required(VERSION 2.8.3)
project(human_walking_detection)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  #turtlesim
)



# Generate messages in the 'msg' folder
add_message_files(
  FILES
    wall.msg
    walls.msg
    Pose.msg
    PoseVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs roscpp  tf
)

###########
## Build ##
###########
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)



add_executable(human_walking_detection src/rosnode.cpp src/main.cpp src/functions.cpp)
target_link_libraries(human_walking_detection ${catkin_LIBRARIES})
#add_dependencies(napoleon_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(human_walking_detection ${catkin_EXPORTED_TARGETS})


#add_executable(human_walking_detection src/rosnode.cpp)
