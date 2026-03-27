cmake_minimum_required(VERSION 3.8)
project(fringe_projection)

# Default to C++14 if no standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(plot_pattern main.cpp src/FringeProcess.cpp src/FringePattern.cpp src/GrayCode.cpp)
target_link_libraries(plot_pattern ${OpenCV_LIBS})

add_executable(debug debug.cpp src/FringeProcess.cpp src/FringePattern.cpp src/GrayCode.cpp src/DebugVisualizer.cpp)
target_link_libraries(debug ${OpenCV_LIBS})