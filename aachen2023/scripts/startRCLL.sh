#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
BTR_DIR="~/git/btr2022"
GAME_DIR="rcap2022"
# TERM="xterm -e"
TERM="gnome-terminal -- bash -c "
 
SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"
PYTHON_DIR="$BTR_DIR/$GAME_DIR/python"
RPLIDAR_DIR="~/catkin_ws/src/rplidar_ros/launch"
ROBOTINO_DIR="~/catkin_ws/src/robotino_node/launch"
# ROBVIEW="robview_interpreter"
for PROGNAME in roscore; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB? /dev/ttyACM?

# gnome-terminal --geometry=105x56 --window\
$TERM "bash -c roscore" &
sleep 1
$TERM "echo rosRcllRefBoxNetwork.sh; sleep 1; cd $SCRIPT_DIR; bash -c ./rosRcllRefBoxNetwork.sh; bash" &
# $TERM "echo robotino.py; sleep 5; cd $PYTHON_DIR; python3 ./robotino.py; bash" &
$TERM "echo robotino_node; sleep 1; cd $ROBOTINO_DIR; roslaunch robotino_node.launch hostname:=127.0.1.1; bash" &
$TERM "echo rplidar.launch; sleep 1; cd $RPLIDAR_DIR; roslaunch rplidar_a3.launch; bash" &
$TERM "echo btr_rplidar.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_rplidar.py; bash" &
$TERM "echo btr_camera.py; sleep 1; cd $PYTHON_DIR; python3 ./btr_camera.py; bash" &
$TERM "echo btr_aruco.py; sleep 2; cd $PYTHON_DIR; python3  ./btr_aruco.py; bash" &
$TERM "echo btr_cobotta_ros.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_cobotta_ros.py; bash" &
# $TERM "echo please power on and run the script for Cobotta.; bash" &
