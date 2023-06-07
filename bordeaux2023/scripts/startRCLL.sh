#!/bin/bash
##
USER_NAME=`id -u -n`
HOME_DIR="/home/$USER_NAME"
REFBOX_DIR="$HOME_DIR/rcll-refbox/bin"
BTR_DIR="$HOME_DIR/git/btr2023"
GAME_DIR="bordeaux2023"
TERM="gnome-terminal -- bash -c "
 
SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"
PYTHON_DIR="$BTR_DIR/$GAME_DIR/python"
PICTURES_DIR="$BTR_DIR/$GAME_DIR/pictures"
RPLIDAR_DIR="$HOME_DIR/catkin_ws/src/rplidar_ros/launch"
ROBOTINO_DIR="$HOME_DIR/catkin_ws/src/robotino_node/launch"

for PROGNAME in roscore; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB? /dev/ttyACM? /dev/video?

pushd `dirname $0`
if [ ! -d $PICTURES_DIR ]; then
	ls $PICTURES_DIR
	mkdir $PICTURES_DIR
fi

$TERM "bash -c roscore" &
sleep 1
$TERM "echo rosRcllRefBoxNetwork.sh; sleep 1; cd $SCRIPT_DIR; bash -c ./rosRcllRefBoxNetwork.sh; bash" &
$TERM "echo robotino_node; sleep 1; cd $ROBOTINO_DIR; roslaunch robotino_node.launch hostname:=127.0.1.1; bash" &
$TERM "echo rplidar.launch; sleep 1; cd $RPLIDAR_DIR; roslaunch rplidar_a3.launch; bash" &
$TERM "echo btr_rplidar.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_rplidar.py; bash" &
$TERM "echo btr_camera.py; sleep 1; cd $PYTHON_DIR; python3 ./btr_camera.py; bash" &
$TERM "echo btr_aruco.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_aruco.py; bash" &
$TERM "echo btr_cobotta_ros.py; sleep 2; cd $PYTHON_DIR; python3 ./btr_cobotta_ros.py; bash" &
popd
