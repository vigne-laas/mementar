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


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MementarEvent.msg
   StampedString.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  MementarService.srv
  MementarEventSubscription.srv
  MementarEventUnsubscription.srv
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
  src/core/archiving_compressing/binaryManagement/BitFileGenerator.cpp
  src/core/archiving_compressing/binaryManagement/BitFileGetter.cpp
  src/core/archiving_compressing/compressing/LzCompress.cpp
  src/core/archiving_compressing/compressing/LzUncompress.cpp
  src/core/archiving_compressing/compressing/Huffman.cpp
  src/core/archiving_compressing/archiving/Header.cpp
  src/core/archiving_compressing/archiving/Archive.cpp
)

add_library(mementar_core_lib
  src/core/EpisodicTree/CompressedLeaf.cpp
  src/core/EpisodicTree/CompressedLeafNode.cpp
  src/core/EpisodicTree/Context.cpp
  src/core/EpisodicTree/CompressedLeafSession.cpp
  src/core/EpisodicTree/CompressedLeafNodeSession.cpp
  src/core/EpisodicTree/ArchivedLeaf.cpp
  src/core/EpisodicTree/ArchivedLeafNode.cpp
)
target_link_libraries(mementar_core_lib
  mementar_compression_lib
  pthread
)

##############################
## EVENTS
##############################

add_library(mementar_events_lib
  src/core/Events/Subscription.cpp
  src/core/Events/EventsManager.cpp
)
target_link_libraries(mementar_events_lib ${catkin_LIBRARIES})
add_dependencies(mementar_events_lib ${catkin_EXPORTED_TARGETS} mementar_gencpp)

##############################
## API
##############################

add_library(mementar_lib
  src/API/clients/ManagerClient.cpp
  src/API/clients/ClientBase.cpp
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
