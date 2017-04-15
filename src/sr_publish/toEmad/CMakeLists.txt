project(sr_publish)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../sr_slam)

add_library(sr_io SR_reader.cpp realsense_reader.cpp SR_interface.cpp SR_tcp_socket.cpp SR_writer.cpp realsense_writer.cpp cam_model.cpp)
target_link_libraries(sr_io ${catkin_LIBRARIES} ${OpenCV_LIBS} mesasr)

## ROS sr_sender and sr_receiver 
add_executable(sr_publisher sr_publisher.cpp)
target_link_libraries(sr_publisher sr_io)

# add_executable(sr_subscriber sr_subscriber.cpp cam_model.cpp)
add_executable(sr_subscriber sr_subscriber.cpp)
target_link_libraries(sr_subscriber sr_io)


