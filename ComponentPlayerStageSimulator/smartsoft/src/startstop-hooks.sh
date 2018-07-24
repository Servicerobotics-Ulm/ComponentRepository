#!/bin/bash

case "$1" in

pre-start)
	echo "Triggering pre-start hooks FROM COMPONENT SmartPlayerStageSimulator ..."
	# Insert commands you want to call prior to starting the components
	
	echo -e "\n\n\n"
	$SMART_ROOT_ACE/repos/DataRepository/player-stage/start-player-stage-world.sh
;;

post-start)
	echo "Triggering post-start hooks FROM COMPONENT SmartPlayerStageSimulator ..."
	# Insert commands you want to call after all components were started
;;

pre-stop)
	echo "Triggering pre-stop hooks FROM COMPONENT SmartPlayerStageSimulator ..."
	# Insert commands you want to call before stopping all components
;;

post-stop)
	echo "Triggering post-stop hooks FROM COMPONENT SmartPlayerStageSimulator ..."
	# Insert commands you want to call after all components were stopped
	
	killall player
;;

*)
	echo "ERROR in $0: no such hook '$1'. Usage: $0 pre-start|post-start|pre-stop|post-stop"
;;

esac
