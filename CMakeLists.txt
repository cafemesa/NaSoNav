cmake_minimum_required(VERSION 3.0.2)
project(social_navigation_testbed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
  gazebo_msgs
  std_msgs
  tf
  rvo2
  pcl_conversions
  pcl_ros
  message_generation
)

catkin_package(
  CATKIN_DEPENDS tf rvo2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_install_python(PROGRAMS
scripts/run_simulation.py
scripts/run_experiment.py
scripts/sacadrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(src)

add_executable(create_simulated_environmets src/create_simulated_environmets.cpp)
target_link_libraries(create_simulated_environmets ${catkin_LIBRARIES})

add_executable(store_annotations src/store_annotations.cpp)
target_link_libraries(store_annotations ${catkin_LIBRARIES})

add_executable(store_metrics_model src/store_metrics_model.cpp)
target_link_libraries(store_metrics_model ${catkin_LIBRARIES})

add_executable(robot_controller src/robot_controller.cpp)
target_link_libraries(robot_controller ${catkin_LIBRARIES})
