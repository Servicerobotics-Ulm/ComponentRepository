# target configurations for ComponentFileMoverROS1InterfacesExtension

# target configurations for ComponentFileMoverRestInterfacesExtension

# target configurations for PlainOpcUaComponentFileMoverExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

