
project(sr_slam)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sr_slam)

add_executable(test_quaternion test_quaternion.cpp)
target_link_libraries(test_quaternion ${catkin_LIBRARIES})

add_executable(test_normal test_normal.cpp)
target_link_libraries(test_normal ${catkin_LIBRARIES})

#library directory set
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# link_directories(${LIBRARY_OUTPUT_PATH})

# build the sr_plane library
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(SR_PLANE_SRC glob_def.cpp point_xyz.cpp plane.cpp plane_set.cpp cam_observation.cpp)
add_library(sr_plane ${SR_PLANE_SRC})
target_link_libraries(sr_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_plane ${SR_PLANE_SRC} test_plane.cpp)
add_executable(test_plane test_plane.cpp)
# target_link_libraries(test_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(test_plane sr_plane)

# add_executable(test_plane2 ${SR_PLANE_SRC} test_plane2.cpp cam_observation.cpp)
add_executable(test_plane2  test_plane2.cpp)
# target_link_libraries(test_plane2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(test_plane2 sr_plane)

add_executable(test_floor test_floor.cpp)
target_link_libraries(test_floor sr_plane sr_io sr_slam sr_slam_plane)

