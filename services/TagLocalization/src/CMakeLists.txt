add_executable(
    TagLocalization

    main.cpp
)

target_link_libraries(
    TagLocalization

	TagDetector
	${OpenCV_LIBS}
)

target_compile_options (TagLocalization PUBLIC -O3)

