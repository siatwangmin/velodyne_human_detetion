cmake_minimum_required(VERSION 2.8.3)
project(velodyne_detect_person)

FIND_LIBRARY(ice_lib Ice)
FIND_LIBRARY(iceUtil_lib IceUtil)
FIND_PATH(ice_path Ice)
FIND_PATH(iceUtil_path IceUtil)

IF(NOT ice_path)
  MESSAGE(FATAL_ERROR "** Librería de Ice no encontrada")
ENDIF()
IF(NOT iceUtil_path)
  MESSAGE(FATAL_ERROR "** Librería de IceUtil no encontrada")
ENDIF()

INCLUDE_DIRECTORIES(
  ${ice_path}
  ${iceUtil_path}
)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
  PCL REQUIRED
  dynamic_reconfigure
)

add_message_files(FILES pointCloudVector.msg)
generate_messages(DEPENDENCIES sensor_msgs)
#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
  #...
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(HEADERS
        "include/PersonPosition.h"
        "include/AriaMapInformation.h"
)
SET(SOURCE
        "src/PersonPosition.cpp"
        "src/AriaMapInformation.cpp"
)

add_executable(extract_clusters src/extract_clusters.cpp)
target_link_libraries(extract_clusters ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(extract_clusters velodyne_detect_person_generate_messages_cpp)

add_executable(set_static_background src/set_static_background.cpp)
target_link_libraries(set_static_background ${catkin_LIBRARIES})
add_dependencies(set_static_background velodyne_detect_person_generate_messages_cpp)

add_executable(set_dynamic_background src/set_dynamic_background.cpp)
target_link_libraries(set_dynamic_background ${catkin_LIBRARIES})
add_dependencies(set_dynamic_background velodyne_detect_person_generate_messages_cpp)

add_executable(find_person src/find_person.cpp ${HEADERS} ${SOURCE})
target_link_libraries(find_person ${catkin_LIBRARIES} ${ice_lib} ${iceUtil_lib})
add_dependencies(find_person velodyne_detect_person_generate_messages_cpp)

add_executable(tf_publisher src/tf_broadcaster.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})
add_dependencies(tf_publisher velodyne_detect_person_generate_messages_cpp)

add_executable(robot_data src/robot_data.cpp ${HEADERS} ${SOURCE})
target_link_libraries(robot_data ${catkin_LIBRARIES} ${ice_lib} ${iceUtil_lib})
add_dependencies(robot_data velodyne_detect_person_generate_messages_cpp)

add_executable(pcl_test src/pcl_test.cpp ${HEADERS} ${SOURCE})
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${ice_lib} ${iceUtil_lib} ${PCL_LIBRARIES})
add_dependencies(pcl_test velodyne_detect_person_generate_messages_cpp)


# for dynamic reconfigure
add_executable(dynamic_reconfigure_node src/server.cpp)

# make sure configure headers are built before any node using them
add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)

# for dynamic reconfigure
target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})
