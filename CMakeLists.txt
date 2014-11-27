cmake_minimum_required(VERSION 2.8.3)
project(mir_rockin_refbox)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    raw_refbox_comm
    roscpp
    std_msgs
)

find_package(Protobuf REQUIRED)

add_definitions("-std=c++0x")
find_package(Boost REQUIRED)

add_message_files(
  FILES
    BenchmarkState.msg
)



generate_messages(
  DEPENDENCIES
  std_msgs
)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

add_executable(rockin_refbox_ros_node
  ros/src/rockin_refbox_ros_node.cpp
  ros/src/rockin_refbox_ros.cpp
  common/src/rockin_refbox.cpp
)

add_dependencies(rockin_refbox_ros_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rockin_refbox_ros_node
  protobuf_comm
  raw_refbox_msgs
  ${Protobuf_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS rockin_refbox_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
