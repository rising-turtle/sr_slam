PROJECT(client_fusion)


cmake_minimum_required(VERSION 2.4.0) 

    if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)

IF(CMAKE_BUILD_TYPE MATCHES DEBUG) 
    message("debug mode") 
ELSE (CMAKE_BUILD_TYPE MATCHES DEBUG) 
	message("release mode")
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG) 

# set(SURODIR "./3rd/SuroROS-1.3-vs2008")
# set(SURODIR "D:/client_fusion/3rd/SuroROS-1.3-vs2008")
STRING(REGEX REPLACE "\\\\" "/" SURO_DIR $ENV{SURODIR})
# STRING( REGEX REPLACE "\\\\" "/"  SURO_DIR  "$ENV{SURODIR}")

#set(SURO_DIR "C:/Users/liu/Desktop/openni2_recorder/liu/SuroROS-1.3-vs2008")
message("SURO_DIR: ${SURO_DIR}")
set(CMAKE_MODULE_PATH   ${SURO_DIR}/cmake/ ) 

include(${SURO_DIR}/cmake/LinkOS.cmake) 

#############FIND SURO PACKAGE################# 

FIND_PACKAGE(SURO REQUIRED) 

LINK_DIRECTORIES(${SURO_DIR}/lib) 
INCLUDE_DIRECTORIES(${SURO_DIR}/include) 

##### IF YOU WANT TO CHANGE OUTPUT BIN PATH, CHANGE THIS LINE #####

# SET(EXECUTABLE_OUTPUT_PATH ${suroros_test_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

##### IF YOU WANT TO CHANGE OUTPUT LIB PATH, CHANGE THIS LINE #####


# SET(LIBRARY_OUTPUT_PATH ${suroros_test_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# INCLUDE_DIRECTORIES(${suroros_test_SOURCE_DIR}/include)

## add suro files 
include_directories(${PROJECT_SOURCE_DIR}/include)

## add boost
include_directories(${CMAKE_SOURCE_DIR}/3rd/boost_1_42/include)
link_directories(${CMAKE_SOURCE_DIR}/3rd/boost_1_42/lib)

# add self libs
link_directories(${CMAKE_SOURCE_DIR}/lib)

############ USER_CHANGE #######################
#IF YOU WANT TO ADD INCLUDE DIRECTORIES, DO LIKE THIS
#INCLUDE_DIRECTORIES(/usr/include/xxx)


###To Change This NORMAL_INTER_LIB to the lib must be include for all the app and test### 


set(NORMAL_INTER_LIB
#IF YOU WANT TO ADD LIB FOR ALL APP AND TEST
# ADD LIKE THIS
#	xxx_lib 
 
	clients_message
	# suroros_test_messages	####Default Message lib , do not delete this line
)


#############################################

###Remove this line will not auto using namespace suro::platform::core::os
ADD_DEFINITIONS(-DSURO_NAME_SPACE_AUTO=1)

IF(WIN32)
include("CMakeWindows.txt")
ELSE()
include("CMakeLinux.txt")
ENDIF(WIN32)


ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

