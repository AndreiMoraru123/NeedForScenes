cmake_minimum_required(VERSION 3.23)
project(NFS)

set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(play main.cpp Toolkit/tools.cpp
        View/camera.cpp Control/controller.cpp
        Algo/tracker.cpp Sensors/lidar.cpp
        Scene/scene.cpp Objects/obstacle.cpp
        Objects/car.cpp Objects/road.cpp
        Objects/parkingspot.cpp)

target_link_libraries (play ${PCL_LIBRARIES})