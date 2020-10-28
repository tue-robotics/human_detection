cmake_minimum_required(VERSION 2.8.3)
project(human_walking_detection)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  armadillo_matrix
  #turtlesim
)



# Generate messages in the 'msg' folder
add_message_files(
  FILES
    wall.msg
    walls.msg
    Pose.msg
    PoseVel.msg
    transform.msg
    singleTube.msg
    tube.msg
    tubes.msg
    tubesH.msg
    line.msg
    lines.msg
    human.msg
    robot.msg
    hypothesis.msg
    hypotheses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs roscpp tf armadillo_matrix
)

###########
## Build ##
###########
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)



add_executable(human_walking_detection src/rosnode.cpp src/main.cpp src/functions.cpp src/functionsDisretizedMap.cpp)
add_executable(human_walking_detection_process src/toTXT.cpp)
target_link_libraries(human_walking_detection ${catkin_LIBRARIES})
add_dependencies(human_walking_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_walking_detection_process ${catkin_LIBRARIES})
add_dependencies(human_walking_detection_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

