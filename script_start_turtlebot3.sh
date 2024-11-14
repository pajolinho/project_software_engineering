#!/bin/bash

# Start turtlebot3_world
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_gazebo turtlebot3_world.launch"

# Pause the script
sleep 10

# open new terminal and start slam
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"

# Pause the script
sleep 15

# open new terminal to control bot with wasd
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=burger; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
