# target configurations for ComponentRealSenseV2ServerROS1InterfacesExtension

# target configurations for ComponentRealSenseV2ServerRestInterfacesExtension

# target configurations for PlainOpcUaComponentRealSenseV2ServerExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

