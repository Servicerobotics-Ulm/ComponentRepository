#!/bin/bash

# Use this variable to name files that are to be deployed to the data
# instance directory of
# the component (<COMPONENT INSTANCE NAME>_data/) on the target host.
#
# Files can be relative to component project directory or absolute.
# Add one file or directory per line.
#
# Examples:
#
# DEPLOY_LIBRARIES="../bin/libMyLibrary.so"
#
# DEPLOY_COMPONENT_FILES="
# $SMART_ROOT_ACE/myFILE
# $SMART_ROOT_ACE/myFILE2
# "

DEPLOY_LIBRARIES=""
DEPLOY_COMPONENT_FILES="
$SMART_ROOT_ACE/bin/libComponentTCLSequencer.so
smartTCL
smartsoft/src/lispInterface
cl-json
load-smartTCL-only.lisp
load-smartTCL-scenarios.lisp
load-deployment-scenario.lisp
ComponentTCLSequencer.ini.smartTCL-only
"
