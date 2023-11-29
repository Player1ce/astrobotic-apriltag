add_executable(
    TagLocalization

    main.cpp
)

target_link_libraries(
    TagLocalization

	TagDetector
	${OpenCV_LIBS}
)

