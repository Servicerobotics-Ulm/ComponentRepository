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

### Pre-requirement
# Execute this command one time in a Terminal and restart the computer/Virtual Machine to be able to launch Webots software through SmartMDSD.
# echo -e "export WEBOTS_HOME=/home/smartsoft/SOFTWARE/webots" >> ~/.profile

### Paths verification
echo " --------------------------------------------------------"
if [ -z "$WEBOTS_HOME" ]
then
echo " |> Error: \$WEBOTS_HOME is not defined, please define it in ~/.profile."
else
echo " |> WEBOTS_HOME: $WEBOTS_HOME"
fi
echo " |> Webots' configuration is done and libraries are set."
echo " --------------------------------------------------------"
