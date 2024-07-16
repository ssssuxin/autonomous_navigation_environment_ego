cmake_minimum_required(VERSION 3.5)
project(vehicle_simulator)
# EXPL project(tare_planner)生成了这个变量${PROJECT_NAME}
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# EXPL ${PROJECT_SOURCE_DIR}表示当前所在目录

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
set(MY_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  PCL
  gazebo_msgs
  nav_msgs
  tf2_ros
)
## Find catkin macros and libraries


find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
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

add_executable(vehicleSimulator src/vehicleSimulator.cpp)
ament_target_dependencies(vehicleSimulator
${MY_DEPENDENCIES} 
)
## Specify libraries to link a library or executable target against
target_link_libraries(vehicleSimulator  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
# add_executable(vehicleSimulator src/vehicleSimulator.cpp)

# target_link_libraries(vehicleSimulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# install(TARGETS vehicleSimulator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY log/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
# )
# install(DIRECTORY mesh/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
# )
# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
# )
# install(DIRECTORY urdf/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
# )
# install(DIRECTORY world/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
# )

install(TARGETS
  vehicleSimulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/#把存yaml文件的文件夹config移到share
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
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