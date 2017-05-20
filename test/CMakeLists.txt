add_executable(hough_circle_test hough_circle_test.cpp)
target_link_libraries(hough_circle_test ${OpenCV_LIBS})

add_executable(point_mark_test point_mark_test.cpp)
target_link_libraries(point_mark_test 
					  sfm
					  ${OpenCV_LIBS})

add_executable(module_test module_test.cpp)

add_executable(feature feature.cpp)
target_link_libraries(feature ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(Eigen_test Eigen_test.cpp)

add_executable(calibration_test calibration_test.cpp)
target_link_libraries(calibration_test sfm)

add_executable(feature_test feature_test.cpp)
target_link_libraries(feature_test sfm)

add_executable(sparse_test sparse_test.cpp)
target_link_libraries(sparse_test sfm)