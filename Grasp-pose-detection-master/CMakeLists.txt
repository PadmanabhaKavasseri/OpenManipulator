cmake_minimum_required(VERSION 3.0)
project(tflite-demo-app-ros1)

# Store in CMAKE_DEB_HOST_ARCH var the current build architecture
execute_process(COMMAND
  dpkg-architecture
    -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
    CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message( STATUS "host arch: ${CMAKE_DEB_HOST_ARCH}" )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2  -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl  -lrt")
message(STATUS "optional:-std=c++17")

#set D435i on or off
set(ENABLE_REALSENSE_D435 "on")



#  set(OpenCV_DIR "/home/rb5/opencv-wayland/build/")

include_directories(/usr/include/freetype2/)
  include_directories(/home/rb5/tensorflow/)
  include_directories(/home/rb5/tensorflow/tensorflow/lite/tools/make/downloads/flatbuffers/include/)
include_directories(/opt/ros/melodic/include)
link_directories(/opt/ros/melodic/lib)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories( ${OpenCV_INCLUDE_DIRS} )


 ADD_LIBRARY(TFlite_LIB STATIC IMPORTED)
 SET_TARGET_PROPERTIES(TFlite_LIB PROPERTIES
	IMPORTED_LOCATION /root/catkin_ws/src/Grasp-pose-detection-master/libs/libtensorflow-lite.a)


set(SOURCE_FILES 
	main.cpp)
	

add_definitions(-DUSE_REALSENSE_D435)

set(SOURCE_FILES_ALL
   ${SOURCE_FILES}
   Camera_D435i.cpp
   )



add_executable(${PROJECT_NAME}  ${SOURCE_FILES_ALL})
target_link_libraries(${PROJECT_NAME} 
	${OpenCV_LIBS} 
	)
	
target_link_libraries(${PROJECT_NAME} 
	freetype
	pthread
	dl
	roscpp
  	cv_bridge
  	rostime
  	image_transport
  	roscpp_serialization
  	rosconsole
  	rosconsole_bridge
	)


target_link_libraries(${PROJECT_NAME} TFlite_LIB)
target_link_libraries(${PROJECT_NAME} realsense2 )

