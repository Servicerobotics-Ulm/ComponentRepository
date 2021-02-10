# includes for ComponentLaserS300ServerROS1InterfacesExtension

# includes for ComponentLaserS300ServerRestInterfacesExtension

# includes for PlainOpcUaComponentLaserS300ServerExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentLaserS300ServerPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


