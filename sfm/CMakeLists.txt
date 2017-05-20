#set control_point_detect module source file
set(source_file ${source_file} 
		control_point_detect/control_point_detect.cpp)


#set calibration_solution module source file
set(source_file ${source_file}
		calibration_solution/calibration_solution.cpp)

#set feature_detect module source file
set(source_file ${source_file}
		feature_detect/feature_detect.cpp)

set(source_file ${source_file}
		sparse_solution/sparse_solution.cpp)

add_library(sfm ${source_file})
target_link_libraries(sfm ${OpenCV_LIBS} ${CERES_LIBRARIES})