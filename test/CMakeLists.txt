add_executable(hough_circle_test hough_circle_test.cpp)
target_link_libraries(hough_circle_test ${OpenCV_LIBS})

add_executable(point_mark_test point_mark_test.cpp)
target_link_libraries(point_mark_test ${OpenCV_LIBS})