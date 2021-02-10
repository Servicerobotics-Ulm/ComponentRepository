# includes for ComponentKeyboardNavigationROS1InterfacesExtension

# includes for ComponentKeyboardNavigationRestInterfacesExtension

# includes for PlainOpcUaComponentKeyboardNavigationExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentKeyboardNavigationPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


