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
rosdep install --from-paths src -r -y # not working might need to install all the dep manually (esp for multi_arm)
```


