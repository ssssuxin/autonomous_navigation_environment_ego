cmake_minimum_required(VERSION 3.5)
project(local_planner)
# EXPL project(tare_planner)生成了这个变量${PROJECT_NAME}
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# EXPL ${PROJECT_SOURCE_DIR}表示当前所在目录
## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS

#   pcl_ros
# )
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

## Declare executables
add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)
ament_target_dependencies(localPlanner
${MY_DEPENDENCIES} 
)
ament_target_dependencies(pathFollower
${MY_DEPENDENCIES} 
)
target_link_libraries(localPlanner ${PCL_LIBRARIES} )
target_link_libraries(pathFollower ${PCL_LIBRARIES} )
## Specify libraries to link a library or executable target against
# target_link_libraries(localPlanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(pathFollower ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# install(TARGETS localPlanner pathFollower
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS
  localPlanner
  pathFollower
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