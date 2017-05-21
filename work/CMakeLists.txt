add_executable(calibration calibration.cpp)
target_link_libraries(calibration sfm)



add_executable(sparse sparse.cpp)
target_link_libraries(sparse sfm)