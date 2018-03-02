cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

PROJECT(Gazebo)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(NewDiffDrivePlugin SHARED newDiffDrivePlugin.cc)

add_library(PubBasePosePlugin SHARED pubBasePosePlugin.cc)

INSTALL(TARGETS PubBasePosePlugin NewDiffDrivePlugin DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
INSTALL(DIRECTORY "${PROJECT_SOURCE_DIR}/../resources/tiago" DESTINATION "$ENV{HOME}/.gazebo/models")


