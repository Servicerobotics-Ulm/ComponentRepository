#!/bin/bash

# This script provides methods to call custom commands pre/post of starting/stoping the component during launch on the device.
# This script is being executed on the target device where the component is running.
# For example the script can be used to start and stop the morse simulator automatically.

case "$1" in

pre-start)
  echo "Triggering pre-start hooks FROM COMPONENT ComponentWebots ..."
  ### Launch Webots with the defined world and startup options
  #  xterm -title "Webots Simulator" -hold -e bash $WEBOTS_HOME/webots --batch --mode=realtime $PWD/ComponentWebots_data/worlds/world.wbt &
  # two webots processes are very slow, so kill an already existing webots process
  WorldPath=`eval echo \`awk -v FS="WorldPath " 'NF>1{print $2}' ${PWD}/ComponentWebots.ini\``
  if [ ! -e $WorldPath ]
  then
    zenity --warning --title="Fatal error" --text="Worldfile is not found: wrong/missing setting in model > .systemParam > ComponentWebots > General > WorldPath (file $WorldPath is not found)" --width=500;
    exit;
  else
    echo WorldPath file found
  fi
  echo " Starting Webots $WorldPath..."
  while pgrep "webots"
  do
    echo "webots already running"
    # bring the zenity window in front after 5 seconds
    (sleep 5 && wmctrl -F -a "Close Webots" -b add,above) &
    zenity --warning --title="Close Webots" --text "Webots is already running, please close Webots." --width=500
    sleep 2
  done
  # absolute path to worldfile is in ComponentWebots parameter WorldPath, read this from Component.ini
  webots `eval echo \`awk -v FS="WorldPath " 'NF>1{print $2}' ${PWD}/ComponentWebots.ini\`` &
  # Insert commands you want to call prior to starting the components
;;

post-start)
  echo "Triggering post-start hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call after all components were started
;;

pre-stop)
  echo "Triggering pre-stop hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call before stopping all components
;;

post-stop)
  echo "Triggering post-stop hooks FROM COMPONENT ComponentWebots ..."
  # Insert commands you want to call after all components were stopped
  echo "kill webots..."
  killall webots
;;

*)
  echo "ERROR in $0: no such hook '$1'. Usage: $0 pre-start|post-start|pre-stop|post-stop"
;;

esac
