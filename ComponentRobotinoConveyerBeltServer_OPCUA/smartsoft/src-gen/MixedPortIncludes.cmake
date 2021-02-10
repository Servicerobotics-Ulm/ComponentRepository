# includes for ComponentRobotinoConveyerBeltServer_OPCUAROS1InterfacesExtension

# includes for ComponentRobotinoConveyerBeltServer_OPCUARestInterfacesExtension

# includes for PlainOpcUaComponentRobotinoConveyerBeltServer_OPCUAExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentRobotinoConveyerBeltServer_OPCUAPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


