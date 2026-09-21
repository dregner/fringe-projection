cmake_minimum_required(VERSION 3.16)
project(StereoCameraSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Spinnaker SDK installation path
set(SPINNAKER_ROOT "/opt/spinnaker" CACHE PATH "Path to Spinnaker SDK")

# --------------------------------------------------------------------------- #
# Architecture detection: JetsonGPIO only builds on ARM64 kernels              #
# --------------------------------------------------------------------------- #
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "[StereoCameraSystem] ARM64 detected - JetsonGPIO ENABLED")
    set(ENABLE_JETSON_GPIO ON)
else()
    message(STATUS "[StereoCameraSystem] Non-ARM64 host (${CMAKE_SYSTEM_PROCESSOR}) "
                   "- JetsonGPIO DISABLED; cameras will use software trigger fallback")
    set(ENABLE_JETSON_GPIO OFF)
endif()

# Propagate the flag so C++ code can switch behavior at compile time
if(ENABLE_JETSON_GPIO)
    set(GPIO_LIB gpiod)
else()
    set(GPIO_LIB "")
endif()
if(ENABLE_JETSON_GPIO)
    add_compile_definitions(STEREO_HAS_JETSON_GPIO)
    add_compile_definitions(HAS_GPIOD)
endif()

# --------------------------------------------------------------------------- #
# External dependencies                                                         #
# --------------------------------------------------------------------------- #
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# --------------------------------------------------------------------------- #
# Include directories (all sources live inside this project)                   #
# --------------------------------------------------------------------------- #
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPINNAKER_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Link directories for Spinnaker
link_directories(
    ${SPINNAKER_ROOT}/lib
)

# --------------------------------------------------------------------------- #
# Fringe-projection library (pattern generation + phase processing)            #
# --------------------------------------------------------------------------- #
add_library(fringe_projection STATIC
    
    src/FringePattern.cpp
    src/FringeProcess.cpp
    src/GrayCode.cpp
)
target_link_libraries(fringe_projection PUBLIC ${OpenCV_LIBS})

# --------------------------------------------------------------------------- #
# Stereo camera system library                                                  #
# --------------------------------------------------------------------------- #
set(STEREO_LIB_SOURCES
    src/StereoCameraSystem.cpp
    src/GpioController.cpp
)
if(ENABLE_JETSON_GPIO)
    set(GPIO_LIB gpiod)
else()
    set(GPIO_LIB "")
endif()


add_library(stereo_camera_system STATIC ${STEREO_LIB_SOURCES})
target_link_libraries(stereo_camera_system PUBLIC
    Spinnaker
    yaml-cpp
    Threads::Threads
    ${GPIO_LIB}
    ${OpenCV_LIBS}
)

# --------------------------------------------------------------------------- #
# Executables                                                                   #
# --------------------------------------------------------------------------- #

# # Original stereo capture demo
# add_executable(stereo_capture src/main.cpp)
# target_link_libraries(stereo_capture PRIVATE stereo_camera_system)
# set_target_properties(stereo_capture PROPERTIES
#     BUILD_WITH_INSTALL_RPATH TRUE
#     INSTALL_RPATH "${SPINNAKER_ROOT}/lib"
# )

# Integrated fringe-projection + stereo acquisition pipeline
add_executable(stereo_fringe src/stereo_fringe_main.cpp)
target_link_libraries(stereo_fringe PRIVATE
    stereo_camera_system
    fringe_projection
)
set_target_properties(stereo_fringe PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${SPINNAKER_ROOT}/lib"
)
