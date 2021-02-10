CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

FIND_PACKAGE(Open62541CppWrapper 1.0 QUIET)

IF(Open62541CppWrapper_FOUND)
	SET(CMAKE_CXX_STANDARD 14)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})
	FILE(GLOB PLAIN_OPCUA_SRCS ${CMAKE_CURRENT_LIST_DIR}/*.cc)
ENDIF(Open62541CppWrapper_FOUND)