project(sr_slam)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../sr_slam)

add_library(sr_io SR_reader.cpp realsense_reader.cpp SR_interface.cpp SR_tcp_socket.cpp SR_writer.cpp realsense_writer.cpp cam_model.cpp)
target_link_libraries(sr_io ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr)

## ROS sr_sender and sr_receiver 
# add_executable(sr_publisher sr_publisher.cpp SR_reader.cpp SR_interface.cpp SR_tcp_socket.cpp SR_writer.cpp)
add_executable(sr_publisher sr_publisher.cpp)
# target_link_libraries(sr_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr) 
target_link_libraries(sr_publisher sr_io)

# add_executable(sr_subscriber sr_subscriber.cpp cam_model.cpp)
add_executable(sr_subscriber sr_subscriber.cpp)
# target_link_libraries(sr_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr) 
target_link_libraries(sr_subscriber sr_io)

## pcl_display 
# add_executable(sr_pcl_show pcl_mesa.cpp SR_reader.cpp SR_interface.cpp cam_model.cpp SR_tcp_socket.cpp SR_writer.cpp)
add_executable(sr_pcl_show pcl_mesa.cpp)
# target_link_libraries(sr_pcl_show ${catkin_LIBRARIES} mesasr)
target_link_libraries(sr_pcl_show sr_io)

## sr_reader test 
# add_executable(sr_read_test SR_reader.cpp SR_test.cpp)
add_executable(sr_read_test SR_test.cpp)
# target_link_libraries(sr_read_test ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr )
target_link_libraries(sr_read_test sr_io)

add_executable(sr_dat_test SR_dat_test.cpp)
target_link_libraries(sr_dat_test sr_io)

add_executable(rs_dat_test RS_dat_test.cpp)
target_link_libraries(rs_dat_test sr_io)

## 2D pos msg publisher 
add_executable(pos_msg pos_msg_pub.cpp pos_msg_main.cpp)
target_link_libraries(pos_msg ${catkin_LIBRARIES})
