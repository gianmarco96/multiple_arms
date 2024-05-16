This repo is heavily based on the 2 following tutorials:
- [Enabling Multi-Robot ARM in Gazebo for ROS2](https://medium.com/@arshad.mehmood/enabling-multi-robot-arm-in-gazebo-for-ros2-dc18981c03c6)
- [An Adaptable Approach to Multi-Robot Navigation in ROS2: Utilizing Turtlebot3 and Nav2](https://medium.com/@arshad.mehmood/a-guide-to-multi-robot-navigation-utilizing-turtlebot3-and-nav2-cd24f96d19c6)


To install this on your machine first make sure you have all the TurtleBot3 dependencies installed as listed in [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) 

You can then clone this repo into your workspace
```
git clone -b ros2_ur https://github.com/gianmarco96/multiple_arms/
```
If you do not have a workspace set up you can create a new one
```
mkdir -p multi_robot_ws/src
cd multi_robot_ws/src
git clone -b ros2_ur https://github.com/gianmarco96/multiple_arms/
```
Now you can build the workspace. Before then, though, make sure you have source the ros2 environment
```
source /opt/ros/foxy/setup.bash
```
Make sure you install all the dependencies
```
cd multi_robot_ws
rosdep install --from-paths src -r -y 
```
Now you are ready to build the environment
```
colcon build --symlink-install
```
**NB:** I have had some issues when building this into a new machine, if this is the case for you as well please contact me and we will spend some time together to set the environemnt together. 
To run the simulation you can run the following commands from terminal:
**Side Note** Given you will have 2 different ROS environments set up (ROS 1 and 2), I recommend **NOT** to add the source command in the bash file but instead source each terminal every time. This is to avoid sourcing a ROS1 and 2 environmnet after each other and risking to concatenate them which casuses issues that are not easy to fix afterwards. So assuming you have sourced the ROS 2 environemnt and workspace you can run the following:
```
ros2 launch gazebo_ros gazebo.launch.py
```
Then in another terminal spwan the turtlebots. (Remember to source the environmnets)
```
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=False enable_rviz:=False
```
For the above I am using a map the blank map that Artur was using in his version of the project. I stored this in the share folder of the turtlebot3_navigation2/map. It does not seem to work if the linked map is the one stored in the local folder. This is something we can fix when we have a bit more time. For the time being please copy the map into the turtlebot3_navigation2 package. You can force the copy using the following command from the local map folder: `sudo cp blank_map_for_tb.yaml /opt/ros/foxy/share/turtlebot3_navigation2/map` and do the same thing for the blank_map_for_tb.pgm file.
Once you have run the command wait for the robots to spawn and for the terminal output to stabilise (a lot of things have been launched at once so you need to make sure everything is launched before you move on).

Finally, you can launch the launch file that spwans the robot arms (once again make sure you source the environments first):
```
ros2 launch multi_robot_arm gazebo_arm.launch.py 
```
Now if you want to run the demo to control the robot you ros run the following script:
```
ros2 run pymoveit2 robot_control.py 
```

