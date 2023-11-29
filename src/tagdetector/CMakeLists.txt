add_executable(
	TagDetector

	tag_detector.cpp
)

target_link_libraries(
	TagDetector
	
	AprilTags
	${OpenCV_LIBS})
