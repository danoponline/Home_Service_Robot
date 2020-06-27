#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/Home_Service_Robot/turtlebot_simulator/turtlebot_gazebo/worlds/playground.world;
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch"
