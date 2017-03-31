if(NOT OPENCV_HIGHGUI_FOUND)
    message(FATAL_ERROR "OpenCV compiled without support for highgui, cannot compile!")
endif()

add_executable( first try.cpp)
target_link_libraries( first chilitags_static )
target_link_libraries( first ${OpenCV_LIBS} )


