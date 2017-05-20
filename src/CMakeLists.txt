add_library(control_point_detect control_point_detect.cpp)
target_link_libraries(control_point_detect ${OpenCV_LIBS})

add_library(calibration_solution calibration_solution.cpp)
target_link_libraries(calibration_solution control_point_detect ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_library(feature_detect feature_detect.cpp)
target_link_libraries(feature_detect control_point_detect ${OpenCV_LIBS} ${CERES_LIBRARIES})