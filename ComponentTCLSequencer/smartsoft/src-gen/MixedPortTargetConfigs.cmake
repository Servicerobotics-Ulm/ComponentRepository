# target configurations for ComponentTCLSequencerROS1InterfacesExtension

# target configurations for ComponentTCLSequencerRestInterfacesExtension

# target configurations for PlainOpcUaComponentTCLSequencerExtension
IF(Open62541CppWrapper_FOUND)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Open62541CppWrapper)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC HAS_OPCUA)
ENDIF(Open62541CppWrapper_FOUND)

