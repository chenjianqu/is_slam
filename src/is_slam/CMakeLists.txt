cmake_minimum_required(VERSION 2.8.3)
project(is_slam)

set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_filters
  tf_conversions
)

#ORB_SLAM的路径
#set(ORB_SLAM_PATH /home/chen/ros/ORB_SLAM2)
set(ORB_SLAM_PATH /home/alex/apps/ORB_SLAM2)

message("ORB_SLAM_PATH = ${ORB_SLAM_PATH} ")

LIST(APPEND CMAKE_MODULE_PATH ${ORB_SLAM_PATH}/cmake_modules)

find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package( PCL REQUIRED)#COMPONENTS )common io visualization filters)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serve_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${ORB_SLAM_PATH}
	${ORB_SLAM_PATH}/include
	${Pangolin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )


set(LIBS 
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	${EIGEN3_LIBS}
	${Pangolin_LIBRARIES}
	${ORB_SLAM_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
	${ORB_SLAM_PATH}/Thirdparty/g2o/lib/libg2o.so
	${ORB_SLAM_PATH}/lib/libORB_SLAM2.so
	 -lboost_system
)


add_library(instance include/is_slam/instance.h src/instance.cpp)
target_link_libraries(instance ${catkin_LIBRARIES} ${LIBS})
add_dependencies(instance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(instancesmap include/is_slam/instancemap.h src/instancemap.cpp)
target_link_libraries(instancesmap ${catkin_LIBRARIES} ${LIBS})
add_dependencies(instancesmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(gridmap include/is_slam/gridmap.h src/gridmap.cpp)
target_link_libraries(gridmap ${catkin_LIBRARIES} ${LIBS})
add_dependencies(gridmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(semanticmap include/is_slam/semanticmap.h src/semanticmap.cpp)
target_link_libraries(semanticmap ${catkin_LIBRARIES} ${LIBS} instancesmap gridmap)
add_dependencies(semanticmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



set(LIBS3
	instancesmap
	semanticmap
  	instance
	gridmap
)


add_executable(orb_slam src/orb_slam.cpp)
target_link_libraries(orb_slam ${catkin_LIBRARIES} ${LIBS})

add_executable(mapper src/mapper.cpp)
target_link_libraries(mapper ${catkin_LIBRARIES} ${LIBS} ${LIBS3})

add_executable(recorder src/recorder.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES} ${LIBS})

add_executable(playsegment src/playsegment.cpp)
target_link_libraries(playsegment ${catkin_LIBRARIES} ${LIBS})

add_executable(playdata src/playdata.cpp)
target_link_libraries(playdata ${catkin_LIBRARIES} ${LIBS})


add_executable(convertor src/convertor.cpp)
target_link_libraries(convertor ${catkin_LIBRARIES} ${LIBS})




