# target configurations for ComponentLaserS300ServerROS1InterfacesExtension

# target configurations for ComponentLaserS300ServerRestInterfacesExtension

# target configurations for PlainOpcUaComponentLaserS300ServerExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

