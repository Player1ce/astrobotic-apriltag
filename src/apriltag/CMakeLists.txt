file(GLOB SOURCE_FILES "src/*.cc")
file(GLOB header_files "AprilTags/*.h")
include_directories(AprilTags . /opt/local/include)
add_library(
	AprilTags 

	${SOURCE_FILES})

target_link_libraries(
	AprilTags 

	${OpenCV_LIBS} v4l2) #-pg) #-fopenmp)
