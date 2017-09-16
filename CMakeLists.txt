project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall -ggdb")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/debug.cpp src/road.cpp src/vehicle.cpp src/ptg.cpp src/map.cpp src/main.cpp )


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
include_directories(src/Eigen-3.3)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
include_directories(src/Eigen-3.3)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS python)
