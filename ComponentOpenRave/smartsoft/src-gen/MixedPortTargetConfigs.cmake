# target configurations for ComponentOpenRaveROS1InterfacesExtension

# target configurations for ComponentOpenRaveROSExtension
IF(EXISTS ${ROS_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROS_LIBS})
ENDIF(EXISTS ${ROS_DIR})

# target configurations for ComponentOpenRaveRestInterfacesExtension

# target configurations for OpcUaBackendComponentGeneratorExtension
IF(SeRoNetSDK_FOUND)
# SeRoNetSDK has to be linked at the minimum (in case the component does not have any ports specified for any reason)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} SeRoNetSDK::SeRoNetSDK)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CommBasicObjectsOpcUa)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CommManipulationPlannerObjectsOpcUa)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CommManipulatorObjectsOpcUa)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CommObjectRecognitionObjectsOpcUa)
ENDIF(SeRoNetSDK_FOUND)

# target configurations for PlainOpcUaComponentOpenRaveExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)
