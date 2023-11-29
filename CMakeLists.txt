cmake_minimum_required(VERSION 3.11)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

project(astrobotic)

# Add all libraries to the include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/src" )

#set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
#set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
#set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

### Targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
