# includes for ComponentRobotinoBaseServerROS1InterfacesExtension

# includes for ComponentRobotinoBaseServerRestInterfacesExtension

# includes for PlainOpcUaComponentRobotinoBaseServerExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentRobotinoBaseServerPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


