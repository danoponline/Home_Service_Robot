#!/bin/sh

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_home_service_robot)/worlds/Danop.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_home_service_robot)/maps/map_Danop.yaml" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch"
