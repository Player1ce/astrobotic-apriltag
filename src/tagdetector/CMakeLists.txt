add_library(
	TagDetector 

	tag_detector.cpp)

target_link_libraries(
	TagDetector
	
	AprilTags
	${OpenCV_LIBS}
	${EIGEN3_INCLUDE_DIR}
	)
