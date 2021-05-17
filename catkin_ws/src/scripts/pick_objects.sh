#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/../../src/map/new_world.world"

xterm -e " cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/../../src/map/my_map.yaml"
xterm -e " cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep  5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun pick_objects pick_objects"
