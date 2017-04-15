# cmake_minimum_required(VERSION 2.4.6)
project(sr_slam) 

add_executable(test_cv_K test_cv_K.cpp)
# target_link_libraries(sr_main rgbdslam_v2 ${catkin_LIBRARIES} ${LIBS_LINK} mesasr) 
target_link_libraries(test_cv_K ${catkin_LIBRARIES})

add_executable(test_publish_pose3d test_publish_pose3d.cpp)
target_link_libraries(test_publish_pose3d ${catkin_LIBRARIES})

add_executable(test_tf_quaternion test_tf_quaternion.cpp)
target_link_libraries(test_tf_quaternion ${catkin_LIBRARIES})

add_executable(bin_pcd_2_ply bin_pc_2_ply.cpp)
target_link_libraries(bin_pcd_2_ply ${catkin_LIBRARIES})
