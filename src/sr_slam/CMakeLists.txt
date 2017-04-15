# cmake_minimum_required(VERSION 2.4.6)
project(sr_slam) 

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###########
## Build ##
###########

# test the original rgbdslam 
add_executable(ori_rgbd ori_rgbd.cpp)
target_link_libraries(ori_rgbd rgbdslam_v2 ${LIBS_LINK}) # ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${G2O_LIBRARIES})

### plane-node ###
QT4_WRAP_CPP(QT_MOC_HPP2 graph_plane.h)
set(SLAM_PLANE_SRCS plane_node.cpp ${QT_MOC_HPP2} graph_plane.cpp graph_plane_gui.cpp feature_detector.cpp gyro_euler.cpp)
add_library(sr_slam_plane ${SLAM_PLANE_SRCS})
target_link_libraries(sr_slam_plane sr_slam sr_io sr_plane)
add_executable(sr_main_plane main_plane.cpp ${SLAM_PLANE_SRCS})

# add_executable(sr_main_plane main_plane.cpp)
# target_link_libraries(sr_main_plane sr_slam sr_io sr_plane)
# target_link_libraries(sr_main_plane sr_slam_plane)

# this definition must be consistent with CMakeLists in rgbdslam_v2
# add_definitions(-DUSE_PCL_ICP)

# test the swiss ranger slam 
QT4_WRAP_CPP(QT_MOC_HPP OpenniWrapper.h graph_wrapper.h)
set(SR_SLAM_SRCS ${QT_MOC_HPP} NodeWrapper.cpp OpenniWrapper.cpp paramSrvMi.cpp cam_model.cpp graph_wrapper.cpp imu_reader.cpp)
add_library(sr_slam ${SR_SLAM_SRCS} ${SLAM_PLANE_SRCS})
target_link_libraries(sr_slam sr_io sr_plane rgbdslam_v2 ${catkin_LIBRARIES} ${LIBS_LINK} ${G2O_LIBRARIES} mesasr)

# add_executable(sr_main main.cpp ${QT_MOC_HPP} NodeWrapper.cpp OpenniWrapper.cpp paramSrvMi.cpp cam_model.cpp graph_wrapper.cpp
#                imu_reader.cpp)
add_executable(sr_main main.cpp)
# target_link_libraries(sr_main rgbdslam_v2 ${catkin_LIBRARIES} ${LIBS_LINK} mesasr) 
target_link_libraries(sr_main sr_slam)

## normally used in offline SLAM process 
add_executable(sr_main_plane main_plane.cpp)
target_link_libraries(sr_main_plane sr_slam)

## imu-gui test for 
add_executable(gyro_show main_gyro_show.cpp gl_cubic.cpp ros_gl_cube.cpp)
target_link_libraries(gyro_show sr_slam glut) 

## imu-gui test for display 
add_executable(test_gyro_display test_gyro_display.cpp gl_cubic.cpp ros_gl_cube.cpp)
target_link_libraries(test_gyro_display ${catkin_LIBRARIES} GLU glut)

add_executable(tmp_display cubic.c set_ticker.c)
target_link_libraries(tmp_display GLU glut)

#############
## Testing ##
#############

add_executable(publish_g2o test_publish_g2o.cpp)
target_link_libraries(publish_g2o sr_slam)

#### for test vro ####
add_executable(test_vo_rs test_vo_rs.cpp)
target_link_libraries(test_vo_rs sr_slam)

add_executable(test_vro test_vro.cpp)
target_link_libraries(test_vro sr_slam)

add_executable(test_ros test_ros.cpp )
target_link_libraries(test_ros sr_slam)

add_executable(test_vro_img test_vo_img.cpp)
target_link_libraries(test_vro_img sr_slam)

#### for test sift ####
add_executable(test_sift test_sift.cpp)
target_link_libraries(test_sift sr_slam)

#### for test G2O using EdgeSE2Line #### 
include_directories(/home/davidz/work/g2o_test/src)
link_directories(/home/davidz/work/g2o_test/lib)
add_executable(test_g2o_edgese2line test_g2o_edgese2line.cpp)
target_link_libraries(test_g2o_edgese2line g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons)

#### for test se3prior jacobian function #### 
add_executable(test_g2o_prior3d_jacobian test_g2o_prior3d_jacobian.cpp)
target_link_libraries(test_g2o_prior3d_jacobian g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons)

#### for test se3euler ####
add_executable(test_g2o_euler3 g2o_copy_graph.cpp test_g2o_euler3.cpp)
target_link_libraries(test_g2o_euler3 g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons)

#### 1.2 for test_g2o_prior3d : replace the 3d xy, with 2d's xy, generate replace_result.g2o #### 
add_executable(test_g2o_prior3d g2o_copy_graph.cpp test_g2o_prior3d.cpp)
target_link_libraries(test_g2o_prior3d g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons )

#### 1.1 for test plane g2o : incrementally use floor plane to optimize the graph #### 
add_executable(test_plane_3d test_plane_3d.cpp)
target_link_libraries(test_plane_3d g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons sr_plane sr_timer ${PCL_LIBRARIES})

#### 2.1 for test plane g2o : incrementatlly use multiple plane to optimize the graph #### 
add_executable(test_mul_plane_3d test_mul_plane_3d.cpp)
target_link_libraries(test_mul_plane_3d g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons sr_plane sr_timer ${PCL_LIBRARIES})

### 1.3/2.2 for test map3d using g2o : generate octomap using g2o/pcds ####
add_executable(test_map3d_g2o test_map_g2o.cpp)
target_link_libraries(test_map3d_g2o g2o_edge_zhadds ${G2O_LIBRARIES} g2o_types_slam3d_addons sr_plane ${PCL_LIBRARIES} octomap_local ${OCTOMAP_LIBS})

### test ground truth result, generate a point cloud using the ground truth data
add_executable(test_gt_pc test_gt.cpp)
target_link_libraries(test_gt_pc sr_slam ${PCL_LIBRARIES})

### receive 3d pos and publish 2d given specific floor plan ### 
add_executable(publish_3d_2d publish_3d_2d.cpp)
target_link_libraries(publish_3d_2d ${catkin_LIBRARIES})

### compute trajectory length ###
add_executable(comp_traj_len test_traj_length.cpp)
target_link_libraries(comp_traj_len ${G2O_LIBRARIES} ${catkin_LIBRARIES})

### clip z error using floor plane ###
add_executable(clip_z test_clip_z.cpp) 
target_link_libraries(clip_z ${G2O_LIBRARIES} ${catkin_LIBRARIES})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sr_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
