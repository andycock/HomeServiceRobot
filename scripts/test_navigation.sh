#!/bin/sh

#Add home environment
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/map/home.world

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &

sleep 3

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 3

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch "