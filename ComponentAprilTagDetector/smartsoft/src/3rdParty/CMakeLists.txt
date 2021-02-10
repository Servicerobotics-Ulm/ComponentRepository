CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
project(apriltag C CXX)

FILE(GLOB_RECURSE COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/apriltag/common/*.c)
FILE(GLOB_RECURSE TAG_FAMILY_SRCS ${CMAKE_CURRENT_LIST_DIR}/apriltag/tag_families/*.c)

SET(APRILTAG_SRCS 
		${CMAKE_CURRENT_LIST_DIR}/apriltag/apriltag.c 
		${CMAKE_CURRENT_LIST_DIR}/apriltag/apriltag_pose.c 
		${CMAKE_CURRENT_LIST_DIR}/apriltag/apriltag_quad_thresh.c 
	)
		
add_library(${PROJECT_NAME} STATIC ${TAG_FAMILY_SRCS} ${APRILTAG_SRCS} ${COMMON_SRCS} )
target_include_directories(${PROJECT_NAME} 	
	PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/apriltag/common/
	${CMAKE_CURRENT_LIST_DIR}/apriltag/tag_families/		
	)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
