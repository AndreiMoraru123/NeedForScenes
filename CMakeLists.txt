cmake_minimum_required(VERSION 3.23)
project(NFS)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_PREFIX_PATH "C:/Users/Andrei/vcpkg/installed/x64-windows/share")
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.12 EXACT REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(play play.cpp View/camera.cpp Objects/car.cpp)
target_link_libraries (play ${PCL_LIBRARIES})