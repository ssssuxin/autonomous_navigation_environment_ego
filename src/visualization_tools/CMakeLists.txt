cmake_minimum_required(VERSION 3.5)
project(visualization_tools)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  PCL
  # visualization_msgs
  nav_msgs
)

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   rospy
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
# )

# find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

# catkin_package(
#   CATKIN_DEPENDS
#   rospy
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # ${catkin_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)


add_executable(visualizationTools src/visualizationTools.cpp)
ament_target_dependencies(visualizationTools
${MY_DEPENDENCIES} 
)
target_link_libraries(visualizationTools ${PCL_LIBRARIES} )
## Declare executables
# add_executable(visualizationTools src/visualizationTools.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(visualizationTools ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  visualizationTools
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/#把存yaml文件的文件夹config移到share
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
##1.自身工程的.h库
ament_package()
