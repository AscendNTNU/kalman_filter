cmake_minimum_required(VERSION 2.8.3)

##CHANGE THIS TO YOUR PACKAGE NAME
project(kalman_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


file(GLOB kalman_src "src/kalman/kalman.cpp")

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ascend_msgs
    geometry_msgs
    roscpp
)



## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ascend_msgs geometry_msgs roscpp
)



## Includes headers in include folder
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(measurement_publisher src/measurement.cpp)
add_executable(kalman_filter src/main.cpp src/Kalman.cpp)

target_link_libraries(kalman_filter ${catkin_LIBRARIES})
target_link_libraries(measurement_publisher ${catkin_LIBRARIES})

add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})