cmake_minimum_required(VERSION 2.8.3)
project(human_intention_prediction)

set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  armadillo_matrix
  hip_msgs
  #turtlesim
)



# Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#    wall.msg
#    walls.msg
#    Pose.msg
#    PoseVel.msg
#    transform.msg
#    singleTube.msg
#    tube.msg
#    tubes.msg
#    tubesH.msg
#    line.msg
#    lines.msg
#    human.msg
#    robot.msg
#    hypothesis.msg
#    hypotheses.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs roscpp tf armadillo_matrix hip_msgs
)

###########
## Build ##
###########
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_executable(human_intention_prediction src/rosnode.cpp src/main.cpp src/functions.cpp src/functionsDisretizedMap.cpp src/associationMatrix.cpp src/KalmanFilter.cpp)
target_link_libraries(human_intention_prediction -O2 blas lapack ${catkin_LIBRARIES})
add_dependencies(human_intention_prediction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(human_intention_prediction_process src/toTXT.cpp)
target_link_libraries(human_intention_prediction_process ${catkin_LIBRARIES})
add_dependencies(human_intention_prediction_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
