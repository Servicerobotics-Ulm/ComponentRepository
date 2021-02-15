# includes for ComponentRobotinoIRServerROS1InterfacesExtension

# includes for ComponentRobotinoIRServerRestInterfacesExtension

# includes for PlainOpcUaComponentRobotinoIRServerExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentRobotinoIRServerPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


