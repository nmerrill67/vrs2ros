cmake_minimum_required(VERSION 3.16)

project(vrs2ros)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag geometry_msgs sensor_msgs nav_msgs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif ()
find_package(vrslib REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag sensor_msgs
    INCLUDE_DIRS src
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall")


include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    vrs::vrslib
)

add_executable(vrs2rosbag src/vrs2rosbag.cpp)
target_link_libraries(vrs2rosbag ${thirdparty_libraries})
