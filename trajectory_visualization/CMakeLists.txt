cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  message_generation
)

## Generate custom service messages
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate messages with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare the package dependencies
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs nav_msgs tf2_ros message_runtime
)

###########
## Build ##
###########

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add Trajectory Publisher and Saver Node
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_dependencies(trajectory_publisher_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_publisher_saver ${catkin_LIBRARIES})

## Add Trajectory Reader and Publisher Node
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
add_dependencies(trajectory_reader_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_reader_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install executables
install(TARGETS trajectory_publisher_saver trajectory_reader_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install service files
install(FILES
  srv/SaveTrajectory.srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
)

#############
## Testing ##
#############

# Uncomment if unit testing is required
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_visualization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

