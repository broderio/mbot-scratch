cmake_minimum_required(VERSION 3.25)

project(scratch_bridge)

set(CMAKE_CXX_STANDARD 20)

set(Python3_EXECUTABLE "/usr/local/bin/python3")
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost REQUIRED)

add_executable(bridge src/bridge.cpp 
                      src/serial.cpp 
                      src/server.cpp)

target_include_directories(bridge PRIVATE include 
                                          ${Python3_INCLUDE_DIRS} 
                                          ${Boost_INCLUDE_DIRS})

target_link_libraries(bridge ${Python3_LIBRARIES} 
                             ${Boost_LIBRARIES})