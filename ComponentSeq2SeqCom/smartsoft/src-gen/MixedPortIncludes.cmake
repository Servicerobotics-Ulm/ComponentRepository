# includes for ComponentSeq2SeqComROS1InterfacesExtension

# includes for ComponentSeq2SeqComRestInterfacesExtension

# includes for PlainOpcUaComponentSeq2SeqComExtension
GET_FILENAME_COMPONENT(PlainOPCUA_DIR "${PROJECT_SOURCE_DIR}/../plainOpcUa" REALPATH)
IF(EXISTS ${PlainOPCUA_DIR})
INCLUDE("${PlainOPCUA_DIR}/src-gen/ComponentSeq2SeqComPlainOpcUa.cmake")
LIST(APPEND FURTHER_SRCS ${PLAIN_OPCUA_SRCS})
ENDIF(EXISTS ${PlainOPCUA_DIR})


