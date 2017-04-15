project(sr_slam)

set(LSD_SLAM_SRC "/home/davidz/work/ros_hydro_ws/src/lsd_slam/lsd_slam_core/src")
message("STATUS LSD_SLAM_SRC: ${LSD_SLAM_SRC}")
include_directories(${LSD_SLAM_SRC})
include_directories(${LSD_SLAM_SRC}/../thirdparty/Sophus)

set(SR_LSD_SLAM_SRCS sr_lsd_slam.cpp sr_depth_map.cpp)
add_library(sr_lsd_slam ${SR_LSD_SLAM_SRCS})
target_link_libraries(sr_lsd_slam lsdslam)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sr_publish)

add_executable(main_sr_lsd main_sr_lsd.cpp)
target_link_libraries(main_sr_lsd sr_lsd_slam ${catkin_LIBRARIES} sr_io)

set(DPT_LSD_SLAM_SRCS dpt_lsd_slam.cpp dpt_lsd_slam_ori.cpp dpt_map.cpp dpt_map_se3.cpp)
add_library(dpt_lsd_slam ${DPT_LSD_SLAM_SRCS})
target_link_libraries(dpt_lsd_slam lsdslam)

add_executable(main_live_dpt_lsd main_live_dpt_lsd.cpp live_slam_wrapper.cpp)
target_link_libraries(main_live_dpt_lsd dpt_lsd_slam)

add_executable(main_imgs_dpt_lsd main_imgs_dpt_lsd.cpp)
target_link_libraries(main_imgs_dpt_lsd dpt_lsd_slam ${catkin_LIBRARIES} sr_io)

add_executable(main_imgs_dpt_with_gt main_img_dpt_with_gt.cpp)
target_link_libraries(main_imgs_dpt_with_gt dpt_lsd_slam ${catkin_LIBRARIES} sr_io)

add_executable(main_show_match_points main_show_match_points.cpp)
target_link_libraries(main_show_match_points dpt_lsd_slam ${catkin_LIBRARIES} sr_slam)

add_executable(test_graph test_graph.cpp)
target_link_libraries(test_graph dpt_lsd_slam ${catkin_LIBRARIES})
