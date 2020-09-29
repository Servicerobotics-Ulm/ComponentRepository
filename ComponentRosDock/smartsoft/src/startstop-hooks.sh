#!/bin/bash

# This script provides methods to call custom commands pre/post of starting/stoping the component during launch on the device. 
# This script is being executed on the target device where the component is running. 
# For example the script can be used to start and stop the morse simulator automatically.

case "$1" in

pre-start)
	echo "Triggering pre-start hooks FROM COMPONENT ComponentRosDock ..."
	# Insert commands you want to call prior to starting the components

	# check if mojin_seronet is sourced
	if [[ $ROS_PACKAGE_PATH != *"mojin_seronet"* ]]; then
		echo "\nMake sure the catkin workspace including mojin_seronet is sourced! Current package path is: $ROS_PACKAGE_PATH \n"
	fi
	roslaunch seronet_dock seronet_dock.launch
;;

post-start)
	echo "Triggering post-start hooks FROM COMPONENT ComponentRosDock ..."
	# Insert commands you want to call after all components were started
;;

pre-stop)
	echo "Triggering pre-stop hooks FROM COMPONENT ComponentRosDock ..."
	# Insert commands you want to call before stopping all components
;;

post-stop)
	echo "Triggering post-stop hooks FROM COMPONENT ComponentRosDock ..."
	# Insert commands you want to call after all components were stopped
;;

*)
	echo "ERROR in $0: no such hook '$1'. Usage: $0 pre-start|post-start|pre-stop|post-stop"
;;

esac
