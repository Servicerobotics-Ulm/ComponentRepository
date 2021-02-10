#!/bin/bash

# This script provides methods to call custom commands pre/post of starting/stoping the component during launch on the device.
# This script is being executed on the target device where the component is running.
# For example the script can be used to start and stop the morse simulator automatically.

case "$1" in

pre-start)
  echo "Triggering pre-start hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call prior to starting the components
;;

post-start)
  echo "Triggering post-start hooks FROM COMPONENT ComponentWebots ..."
  ### Launch Webots with the defined world and startup options
  echo " Starting Webots..."
  xterm -title "Webots Simulator" -hold -e bash $WEBOTS_HOME/webots --batch --mode=realtime $PWD/ComponentWebots_data/worlds/world.wbt &
  # Insert commands you want to call after all components were started
;;

pre-stop)
  echo "Triggering pre-stop hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call before stopping all components
;;

post-stop)
  echo "Triggering post-stop hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call after all components were stopped
  echo " Webots is closing..."
  killall webots-bin
;;

*)
  echo "ERROR in $0: no such hook '$1'. Usage: $0 pre-start|post-start|pre-stop|post-stop"
;;

esac
