# includes for ComponentRealSenseV2ServerROS1InterfacesExtension

# includes for ComponentRealSenseV2ServerRestInterfacesExtension

# includes for PlainOpcUaComponentRealSenseV2ServerExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentRealSenseV2ServerPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


