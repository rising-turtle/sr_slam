project(sr_slam)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## ROS sr_sender and sr_receiver 
add_executable(sr_publisher sr_publisher.cpp SR_reader.cpp SR_interface.cpp SR_tcp_socket.cpp)
target_link_libraries(sr_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr) 

add_executable(sr_subscriber sr_subscriber.cpp cam_model.cpp)
target_link_libraries(sr_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr) 

## pcl_display 
add_executable(sr_pcl_show pcl_mesa.cpp SR_reader.cpp SR_interface.cpp cam_model.cpp SR_tcp_socket.cpp)
target_link_libraries(sr_pcl_show ${catkin_LIBRARIES} mesasr)

## sr_reader test 
add_executable(sr_read_test SR_reader.cpp SR_test.cpp)
target_link_libraries(sr_read_test ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr )
