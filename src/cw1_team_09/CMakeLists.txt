cmake_minimum_required(VERSION 3.0.2)
project(cw1_team_09) # change your team name here!

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  pcl_ros
  cw1_world_spawner
  pcl_conversions
)

## Find the full PCL package and the required modules.
## Added "features" for normal estimation.
find_package(PCL REQUIRED COMPONENTS filters kdtree ml sample_consensus search segmentation surface features)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
add_service_files(
  FILES
  example.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface 
    tf2 
    tf2_ros 
    pcl_ros 
    cw1_world_spawner
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# (Optional) Link directories for PCL, if needed
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ library
add_library(${PROJECT_NAME}_cw1_class_lib
  src/cw1_class.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME}_cw1_class_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Declare a C++ executable
add_executable(cw1_solution_node src/cw1_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(cw1_solution_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Link your library and executable with both catkin and PCL libraries.
target_link_libraries(${PROJECT_NAME}_cw1_class_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(cw1_solution_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_cw1_class_lib
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# (Installation rules can be added as needed.)

#############
## Testing ##
#############

# (Testing rules can be added as needed.)
