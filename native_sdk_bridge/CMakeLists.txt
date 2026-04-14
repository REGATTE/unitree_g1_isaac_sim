cmake_minimum_required(VERSION 3.16)
project(unitree_g1_native_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(unitree_g1_native_bridge
  src/main.cpp
  src/lowstate_publisher.cpp
)

target_include_directories(unitree_g1_native_bridge
  PRIVATE
    include
)
