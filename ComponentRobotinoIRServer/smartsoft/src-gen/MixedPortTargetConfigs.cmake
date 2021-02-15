# target configurations for ComponentRobotinoIRServerROS1InterfacesExtension

# target configurations for ComponentRobotinoIRServerRestInterfacesExtension

# target configurations for PlainOpcUaComponentRobotinoIRServerExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

