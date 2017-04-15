cmake_minimum_required(VERSION 2.4.6)
project(sr_slam)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

message("STATUS PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  tf
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sr_slam
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf pcl_conversions pcl_ros
  DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

# CXX flags
set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -fPIC -fpermissive -std=c++0x -w")
# cmake_modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

#############################
# Eigen ####################
#############################
 find_package(Eigen)
 set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
 include_directories(${EIGEN_INCLUDE_DIRS})
 add_definitions(${EIGEN_DEFINITIONS})

# PCL #######################
 set(PCL_DIR "/home/davidz/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
 find_package(PCL 1.7 REQUIRED COMPONENTS common io)
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 # add_definitions(${PCL_DEFINITIONS})

# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# for linking to libcxsparse2.2.3
link_directories("/usr/lib/i386-linux-gnu")

#############################
# OpenCV ####################
#############################
 find_package(OpenCV)
 include_directories(${OpenCV_INCLUDE_DIRS})
 link_directories(${OpenCV_LIBRARY_DIRS})

 #########################
 #### libMesaSR 
 ########################
 include_directories(/usr/include)  

###########################
## rgbdslam_v2.so 
###########################
set(ROS_INDIGO_ROOT "/home/davidz/work/ros_hydro_ws")
set(RGBDSLAM_root "${ROS_INDIGO_ROOT}/src/rgbdslam_v2/include")
set(ROS_INDIGO_LIB "${ROS_INDIGO_ROOT}/lib")
include_directories(${RGBDSLAM_root})
link_directories(${ROS_INDIGO_LIB})

## set sift_GPU, this has to be the same with rgbdslam_v2/CMakeLists.txt
#############################
# SiftGPU ###################
#############################
set(USE_SIFT_GPU        1) 
set(ENV{SIFT_GPU_MODE}	2) #CUDA = 1, GLSL = 2
IF (${USE_SIFT_GPU})
    SET(LIBS_LINK ${LIBS_LINK} siftgpu)
ENDIF (${USE_SIFT_GPU})
IF (${USE_SIFT_GPU})
  # add_subdirectory(external)
  # include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/SiftGPU/src)
  set(SIFT_GPU_PATH /home/davidz/work/ros_hydro_ws/src/rgbdslam_v2/external/SiftGPU)
  include_directories(${SIFT_GPU_PATH}/src)
  link_directories(${SIFT_GPU_PATH}/bin)	
  add_definitions(-DUSE_SIFT_GPU)
  add_definitions(-DSIFT_GPU_MODE=$ENV{SIFT_GPU_MODE})
ENDIF (${USE_SIFT_GPU})

###########################
## G2O
###########################

# for FindG2O.cmake
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

# g2o #######################
set(G2O_ROOT "/home/davidz/work/3rdlibs/g2o/install")
# set(G2O_INCLUDE_DIR "${G2O_DIR}/include")
# set(G2O_LIBRARIES "${G2O_DIR}/lib")
find_package(G2O REQUIRED)
set(G2O_INCLUDE_DIRS "${G2O_ROOT}/include" "/usr/include/suitesparse/")
include_directories(${G2O_INCLUDE_DIRS})
set(G2O_LIBRARY_DIRS "${G2O_ROOT}/lib")
link_directories(${G2O_LIBRARY_DIRS})
## g2o_ext_csparse
set(G2O_LIBS g2o_types_slam2d g2o_types_slam2d_addons g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse g2o_stuff g2o_core g2o_csparse_extension g2o_solver_eigen g2o_solver_dense g2o_types_slam3d_addons) 
set(G2O_LIBRARIES ${G2O_LIBS} cholmod cxsparse)

# for linking to libcxsparse2.2.3
link_directories("/usr/lib/i386-linux-gnu")

#Only possible when not subscribing to point cloud topic. Possibly dangerous anyway, but halves storage space per point cloud
#ADD_DEFINITIONS(-DRGB_IS_4TH_DIM) 
#ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)
#ADD_DEFINITIONS(-DHEMACLOUDS) 
ADD_DEFINITIONS(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

#### octomap #### 
set(OCTOMAP_INCLUDE_DIR "/home/davidz/work/ros_hydro_ws/src/octomap_local/src")
set(OCTOMAP_LIBRARIES_DIR "/home/davidz/work/3rdlibs/octomap/lib")
set(OCTOMAP_LIBS octomap octomath)
include_directories(${OCTOMAP_INCLUDE_DIR})
link_directories(${OCTOMAP_LIBRARIES_DIR})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# all the sub-modules in this folder 
add_subdirectory(src)
