cmake_minimum_required(VERSION 2.8.3)
project(mementar)

add_compile_options(-std=c++1z -Wall)
link_libraries(stdc++fs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  pluginlib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MementarOccasion.msg
   StampedString.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  MementarService.srv
  MementarOccasionSubscription.srv
  MementarOcassionUnsubscription.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mementar_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##############################
## Declare a C++ library
##############################
##############################
## ARCHIVING & COMPRESSION
##############################

##############################
## CORE
##############################
add_library(mementar_compression_lib
  src/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.cpp
  src/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.cpp
  src/core/LtManagement/archiving_compressing/compressing/LzCompress.cpp
  src/core/LtManagement/archiving_compressing/compressing/LzUncompress.cpp
  src/core/LtManagement/archiving_compressing/compressing/Huffman.cpp
  src/core/LtManagement/archiving_compressing/archiving/Header.cpp
  src/core/LtManagement/archiving_compressing/archiving/Archive.cpp
)

add_library(mementar_memGraphs_lib
  src/core/memGraphs/Branchs/ValuedNode.cpp
  src/core/memGraphs/Branchs/types/Action.cpp
  src/core/memGraphs/Branchs/types/Fact.cpp
  src/core/memGraphs/Branchs/types/Triplet.cpp
  src/core/memGraphs/Graphs/ActionGraph.cpp
  src/core/memGraphs/Graphs/FactGraph.cpp
)

add_library(mementar_core_lib
  src/core/LtManagement/EpisodicTree/CompressedLeaf.cpp
  src/core/LtManagement/EpisodicTree/CompressedLeafNode.cpp
  src/core/LtManagement/EpisodicTree/Context.cpp
  src/core/LtManagement/EpisodicTree/CompressedLeafSession.cpp
  src/core/LtManagement/EpisodicTree/CompressedLeafNodeSession.cpp
  src/core/LtManagement/EpisodicTree/ArchivedLeaf.cpp
  src/core/LtManagement/EpisodicTree/ArchivedLeafNode.cpp
)
target_link_libraries(mementar_core_lib
  mementar_compression_lib
  mementar_memGraphs_lib
  pthread
)

##############################
## EVENTS
##############################

add_library(mementar_events_lib
  src/core/Occasions/Subscription.cpp
  src/core/Occasions/OccasionsManager.cpp
)
target_link_libraries(mementar_events_lib ${catkin_LIBRARIES})
add_dependencies(mementar_events_lib ${catkin_EXPORTED_TARGETS} mementar_gencpp)

##############################
## DRAWER
##############################

add_library(mementar_drawer_lib
  src/graphical/timeline/ActionReader.cpp
  src/graphical/timeline/EventReader.cpp
  src/graphical/timeline/TimelineDrawer.cpp
)
target_include_directories(mementar_drawer_lib
PUBLIC
 ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mementar_drawer_lib
  mementar_memGraphs_lib
  ${OpenCV_LIBS}
)

##############################
## API
##############################

add_library(mementar_lib
  src/API/TimelineManipulator.cpp
  src/API/TimelinesManipulator.cpp
  src/API/clients/ManagerClient.cpp
  src/API/clients/ClientBase.cpp
  src/API/OccasionsPublisher.cpp
  src/API/OccasionsSubscriber.cpp
)
target_link_libraries(mementar_lib ${catkin_LIBRARIES})
add_dependencies(mementar_lib ${catkin_EXPORTED_TARGETS} mementar_gencpp)

##############################
## INTERFACE
##############################

add_library(mementar_interface
  src/RosInterface.cpp
)
target_link_libraries(mementar_interface mementar_core_lib mementar_events_lib)
target_link_libraries(mementar_interface ${catkin_LIBRARIES})
add_dependencies(mementar_interface ${catkin_EXPORTED_TARGETS} mementar_gencpp)

##############################
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##############################

add_executable(mementar src/nodes/mementar.cpp)
target_link_libraries(mementar mementar_interface)
target_link_libraries(mementar ${catkin_LIBRARIES})
add_dependencies(mementar ${catkin_EXPORTED_TARGETS})

add_executable(mementar_multi src/nodes/mementar_multi.cpp)
target_link_libraries(mementar_multi mementar_interface)
target_link_libraries(mementar_multi ${catkin_LIBRARIES})
add_dependencies(mementar_multi ${catkin_EXPORTED_TARGETS})

add_executable(mementar_timeline src/graphical/timeline/main.cpp)
target_link_libraries(mementar_timeline mementar_core_lib mementar_drawer_lib)
target_link_libraries(mementar_timeline ${catkin_LIBRARIES})
add_dependencies(mementar_timeline ${catkin_EXPORTED_TARGETS})

##############################
## Test executables
##############################

add_executable(event_sub_pub src/test/occasions_sub_pub.cpp)
target_link_libraries(event_sub_pub ${catkin_LIBRARIES})
target_link_libraries(event_sub_pub mementar_lib)

add_executable(graphs src/test/graphs.cpp)
target_link_libraries(graphs ${catkin_LIBRARIES})
target_link_libraries(graphs mementar_memGraphs_lib)


#add_executable(eventLink src/test/EventLink.cpp)
#target_link_libraries(eventLink mementar_core_lib)

##############################
## Install
##############################

install(TARGETS
  mementar_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

##############################
## Tests
##############################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  #add_rostest_gtest(mementar_tester test/library.test src/tests/library.cpp)
  #target_link_libraries(mementar_tester mementar_lib)
endif()
