cmake_minimum_required(VERSION 2.6)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

project(apriltags)
set(APRILTAG_LIBRARY_NAME ApriltagsCPP)

set(CMAKE_INSTALL_PREFIX "/usr/local/" CACHE PATH "Installation prefix")

set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB SOURCE_FILES "src/*.cc")
file(GLOB header_files "AprilTags/*.h")
include_directories(AprilTags . /opt/local/include)
add_library(${APRILTAG_LIBRARY_NAME} ${SOURCE_FILES})
add_subdirectory(example)

target_link_libraries(${APRILTAG_LIBRARY_NAME} ${OpenCV_LIBS} v4l2) #-pg) #-fopenmp)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/${APRILTAG_LIBRARY_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${APRILTAG_LIBRARY_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${APRILTAG_LIBRARY_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${APRILTAG_LIBRARY_NAME}ConfigVersion.cmake"
  VERSION "2.0"
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${APRILTAG_LIBRARY_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${APRILTAG_LIBRARY_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${APRILTAG_LIBRARY_NAME}
)

# Specify the installation directory
install(
    TARGETS ${APRILTAG_LIBRARY_NAME}
    EXPORT ${APRILTAG_LIBRARY_NAME}Targets
    DESTINATION lib
)

install(
    FILES ${header_files}
    DESTINATION include/${APRILTAG_LIBRARY_NAME}/AprilTags
)

install(EXPORT ${APRILTAG_LIBRARY_NAME}Targets
  FILE ${APRILTAG_LIBRARY_NAME}Targets.cmake
  DESTINATION lib/cmake/${APRILTAG_LIBRARY_NAME}
)