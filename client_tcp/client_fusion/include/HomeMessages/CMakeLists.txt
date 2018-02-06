set( ${PROJECT_NAME}_MESSAGES_SRCS 
	#TODO 
	#TO ADD protoc.pb.cc FILE TO HERE
	suroros_testMessage.cpp
	messages_@home_zjupanda_odometer.pb.cc
	messages_@home_zjupanda_laser.pb.cc
	messages_@home_zjupanda_robotspeed.pb.cc
)


ADD_LIBRARY(${NORMAL_INTER_LIB} ${${PROJECT_NAME}_MESSAGES_SRCS})
