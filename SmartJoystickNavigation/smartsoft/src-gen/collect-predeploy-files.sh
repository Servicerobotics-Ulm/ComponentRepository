#!/bin/bash

#source predeploy
source smartsoft/src/predeploy.sh 2>&1

echo "DEPLOY_LIBRARIES=$DEPLOY_LIBRARIES"
echo "DEPLOY_COMPONENT_FILES=$DEPLOY_COMPONENT_FILES"

COMPONENT_NAME="SmartJoystickNavigation"

TMPDIR=$(mktemp -d --suffix=.component-predeploy-files) || exit 1

echo "Temporary directory: $TMPDIR"

############ LIBS ############
for I in $DEPLOY_LIBRARIES; do
	if [ -e "$SMART_ROOT_ACE/bin/$I" ]; then
		FILE="$SMART_ROOT_ACE/bin/$I"
	else
		FILE="$SMART_ROOT_ACE/lib/$I"
	fi
	DEPLOY_LIBRARIES_USER="$DEPLOY_LIBRARIES_USER $FILE"
done

mkdir $TMPDIR/libs

if [ ! "$DEPLOY_LIBRARIES_USER" = "" ]; then
	cp -rv $DEPLOY_LIBRARIES_USER $TMPDIR/libs/ 2>&1
fi

############ USER FILES ############
DEPLOY_COMPONENT_FILES_USER=""
for I in $DEPLOY_COMPONENT_FILES; do
	if ls ./$I > /dev/null 2>&1; then
		DEPLOY_COMPONENT_FILES_USER="$DEPLOY_COMPONENT_FILES_USER ./$I"
	elif ls $I > /dev/null 2>&1; then
		DEPLOY_COMPONENT_FILES_USER="$DEPLOY_COMPONENT_FILES_USER $I"
	fi
done

mkdir $TMPDIR/files

if [ ! "$DEPLOY_COMPONENT_FILES_USER" = "" ]; then
	cp -rv $DEPLOY_COMPONENT_FILES_USER $TMPDIR/files/ 2>&1
	cd $TMPDIR/files
	tar czvf "/tmp/${COMPONENT_NAME}_predeploy_files.tar.gz" *
fi

if [ ! "$DEPLOY_LIBRARIES_USER" = "" ]; then
	cd $TMPDIR/libs
	tar czvf "/tmp/${COMPONENT_NAME}_predeploy_libs.tar.gz" *
fi
