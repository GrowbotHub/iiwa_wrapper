cmake_minimum_required(VERSION 2.8.3)

project(test)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  #growbot_msg
  iiwa_ros
  iiwa_msgs 
  dynamic_reconfigure
  #utils
  #grasp_interface
)

find_package (Eigen3 REQUIRED)

## Generate added messages and services
#generate_messages(DEPENDENCIES iiwa_msgs)

# Generate dynamic reconfigure parameters


catkin_package(
  #INCLUDE_DIRS include
  #INCLUDE_DIRS ../iiwa_stack/iiwa_ros/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure utils grasp_interface growbot_msg iiwa_ros iiwa_msgs
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(test_kuki src/testKuki_node.cpp src/TestKuki.cpp src/iiwa_wrapper.cpp)
target_link_libraries(test_kuki ${catkin_LIBRARIES})
add_dependencies(test_kuki ${PROJECT_NAME}_gencpp)

#add_library(iiwa_wrapper src/iiwa_wrapper.cpp)
#target_link_libraries(test_kuki iiwa_wrapper)

#add_executable(move_potatoes src/movePotatoes_node.cpp src/MovePotatoes.cpp)
#target_link_libraries(move_potatoes ${catkin_LIBRARIES})
#add_dependencies(move_potatoes ${PROJECT_NAME}_gencpp)


## Declare a cpp library
FILE(GLOB SRCS src/TestKuki.cpp)
FILE(GLOB SRCS src/iiwa_wrapper.cpp)
FILE(GLOB INCS include/TestKuki.h)
FILE(GLOB INCS include/iiwa_wrapper.hpp)



add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
