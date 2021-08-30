
cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11
set(CMAKE_BUILD_TYPE Release)
#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
ADD_EXECUTABLE(${PROJECT_NAME}
               main.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ydlidar_driver)
