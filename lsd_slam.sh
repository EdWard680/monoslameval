#! /bin/bash

{
gnome-terminal -x bash -c "roscore;exec bash"
}&
 
sleep 1s
{
gnome-terminal -x bash -c "rosrun lsd_slam_viewer viewer"
}&
 
sleep 1s
{
gnome-terminal -x bash -c "rosrun lsd_slam_core live_slam /image:=/camera/image_raw _calib:=$PWD/morning/camera.cfg"
}


