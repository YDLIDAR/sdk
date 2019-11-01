cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)
include_directories(easylogger)
include_directories(easylogger/inc)
include_directories(easylogger/plugins)



IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
  "easylogger/port/*.c"
  "easylogger/plugins/file/*.c"
  "easylogger/src/*.c"
  "easylogger/plugins/file/*.h"
  "easylogger/src/*.h"
  "easylogger/port/*.h"
  "easylogger/inc/*.h"
)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
  "easylogger/port/*.c"
  "easylogger/plugins/file/*.c"
  "easylogger/src/*.c"
  "easylogger/plugins/file/*.h"
  "easylogger/src/*.h"
  "easylogger/port/*.h"
  "easylogger/inc/*.h"
)

	
ENDIF()

add_subdirectory(samples)

add_library(ydlidar_driver STATIC ${SDK_SRC})
IF (WIN32)
target_link_libraries(ydlidar_driver setupapi Winmm)
ELSE()
target_link_libraries(ydlidar_driver rt pthread)
ENDIF()                    
