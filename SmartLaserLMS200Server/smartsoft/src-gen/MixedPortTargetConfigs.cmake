# target configurations for PlainOpcUaSmartLaserLMS200ServerExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

# target configurations for SmartLaserLMS200ServerROS1InterfacesExtension

# target configurations for SmartLaserLMS200ServerRestInterfacesExtension

