cmake_minimum_required(VERSION 2.8.3)
project(lead_follow)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  cmake_modules
  tf
  nav_msgs
)

find_package(Eigen REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lead_follow
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Declare a C++ library
# add_library(lead_follow
#   src/${PROJECT_NAME}/lead_follow.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lead_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lead_follow_node src/lead_follow_node.cpp)
#add_dependencies(lead_follow_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lead_follow_node
  ${catkin_LIBRARIES}
)

add_executable(tracking_follower
  src/tracking_follower.cpp
)
add_dependencies(tracking_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_follower
  ${catkin_LIBRARIES}
)