# multiple_arms
This repo is heavily based on [MoveIt! Multiple Robot Arms Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/multiple_robot_arms/multiple_robot_arms_tutorial.html). The following steps can be applied to any robot platform, however I decided to stick to the panda arm (as apposed to UR, for instance) because these are more prone to multi-robot control, thanks to their more clear xacro/urdf and the absence of custom kinematics or controller (e.g. scaled_controllers).

## Xacro and URDF
The simulated cell with multiple robots starts with the creation of the xacro file. Once again this step is very similar to the tutorial, I have just added 2 more robots and move all robots closed to each other to make sure they can actually collide (given that the aim of this project is to validate a collision algorithm).

I believe this is self-explainatory but please do not hesitate to contact me if you have any questoins.


<details>

<summary>panda_multiple_arms.xacro</summary>

```
<?xml version="1.0"?>
<robot name="panda_multiple_arms" xmlns:xacro="http://ros.org/wiki/xacro">

    <!-- add arms names prefixes -->
    <xacro:arg name="arm_id_1" default="front_right_arm" />
    <xacro:arg name="arm_id_2" default="front_left_arm" />
    <xacro:arg name="arm_id_3" default="rear_right_arm" />
    <xacro:arg name="arm_id_4" default="rear_left_arm" />

    <!-- load arm/hand models and utils (which adds the robot inertia tags to be Gazebo-simulation ready) -->
    <xacro:include filename="$(find franka_description)/robots/common/utils.xacro" />
    <xacro:include filename="$(find franka_description)/robots/common/franka_arm.xacro" />
    <xacro:include filename="$(find franka_description)/robots/common/franka_hand.xacro" />

    <link name="world"/>

    <!-- box shaped table as base for the 4 Pandas -->
    <link name="base">
        <visual>
            <origin xyz="0 0 0.5" rpy="0 0 0" />
            <geometry>
                <box size="2 2 1" />
            </geometry>
            <material name="White">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0.5" rpy="0 0 0" />
            <geometry>
                <box size="2 2 1" />
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <mass value="20.0"/>
            <inertia ixx="0.001" ixy="0.0" ixz="0.001" iyy="0.0" iyz="0.0" izz="0.001"/>
        </inertial>
        
    </link>

    <joint name="base_to_world" type="fixed">
        <parent link="world"/>
        <child link="base"/>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </joint>
    
    <!-- front right arm with gripper -->
    <xacro:franka_arm arm_id="$(arg arm_id_1)" connected_to="base" xyz="0.5 -0.5 1" gazebo="true" safety_distance="0.03" joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}"/>
    <xacro:franka_hand arm_id="$(arg arm_id_1)" rpy="0 0 ${-pi/4}" connected_to="$(arg arm_id_1)_link8" gazebo="true" safety_distance="0.03" />

    <!-- front left arm with gripper -->
    <xacro:franka_arm arm_id="$(arg arm_id_2)" connected_to="base" xyz="0.5 0.5 1" gazebo="true" safety_distance="0.03" joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}"/>
    <xacro:franka_hand arm_id="$(arg arm_id_2)" rpy="0 0 ${-pi/4}" connected_to="$(arg arm_id_2)_link8" gazebo="true" safety_distance="0.03" />

    <!-- rear right arm with gripper -->
    <xacro:franka_arm arm_id="$(arg arm_id_3)" connected_to="base" xyz="-0.5 -0.5 1" gazebo="true" safety_distance="0.03" joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}"/>
    <xacro:franka_hand arm_id="$(arg arm_id_3)" rpy="0 0 ${-pi/4}" connected_to="$(arg arm_id_3)_link8" gazebo="true" safety_distance="0.03" />

    <!-- rear left arm with gripper -->
    <xacro:franka_arm arm_id="$(arg arm_id_4)" connected_to="base" xyz="-0.5 0.5 1" gazebo="true" safety_distance="0.03" joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}"/>
    <xacro:franka_hand arm_id="$(arg arm_id_4)" rpy="0 0 ${-pi/4}" connected_to="$(arg arm_id_4)_link8" gazebo="true" safety_distance="0.03" />


    <!-- front right arm joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint1" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint2" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint3" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint4" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint5" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint6" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_joint7" transmission="hardware_interface/PositionJointInterface" />

    <!-- front left arm joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint1" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint2" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint3" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint4" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint5" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint6" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_joint7" transmission="hardware_interface/PositionJointInterface" />

    <!-- rear right arm joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint1" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint2" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint3" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint4" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint5" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint6" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_joint7" transmission="hardware_interface/PositionJointInterface" />

    <!-- rear left arm joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint1" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint2" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint3" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint4" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint5" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint6" transmission="hardware_interface/PositionJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_joint7" transmission="hardware_interface/PositionJointInterface" />


    <!-- right hand joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_1)_finger_joint1" transmission="hardware_interface/EffortJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_1)_finger_joint2" transmission="hardware_interface/EffortJointInterface" />

    <!-- left hand joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_2)_finger_joint1" transmission="hardware_interface/EffortJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_2)_finger_joint2" transmission="hardware_interface/EffortJointInterface" />

    <!-- right hand joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_3)_finger_joint1" transmission="hardware_interface/EffortJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_3)_finger_joint2" transmission="hardware_interface/EffortJointInterface" />

    <!-- left hand joints control interface -->
    <xacro:gazebo-joint joint="$(arg arm_id_4)_finger_joint1" transmission="hardware_interface/EffortJointInterface" />
    <xacro:gazebo-joint joint="$(arg arm_id_4)_finger_joint2" transmission="hardware_interface/EffortJointInterface" />


    <!-- load ros_control plugin -->
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/>
    </gazebo>

</robot>
```
</details>

## Disabling collisions
The moveit library automatically checks for collisions of links within the .srds created by default. In our case, in order to validate our algorithm we need to make sure that collision checks for the different robots are disabled. 

When launching the moveit setup assistant, one of the first steps required to generate the .srdf file is the self-collisions disable function. This step is carried out to optmise the collision search by removing the checks for links that are always in collision, never in collision, in collision in the robot's default position, or when the links are adjacent to each other on the kinematic chain. We can use this step to disable collision checks for the 4 different robots we have created in the previous step.

Take your time and disable all collision between links of different robots, make sure you leave the collisions check with base enabled for all robots. *Note:* The figure below shows the matrix generated by the setup assistant automatically and was taken before the collision were manually disabled.

![Alt text](media/CollisionMatrix.png?raw=true "CollisionMatrix")

## Planning Groups
In this step we will generate 9 planning groups
- 4 planning groups for the 4 robots
- 4 planning groups for the 4 grippers
- 1 multi_robot group to control the 4 robots at once

For the robot and gripper planning group follow the instructions on this tutorial, make sure you assign the right joints and links to the right groups. I named my groups as the prefix we used to generate the urdfs (i.e. front_left_arm, front_right_arm, rear_left_arm, rear_right_arm, front_left_hand, front_right_hand, rear_left_hand, rear_right_hand). This makes it easier to check joints and links are in the right group at the end.

### Multi robot group
This group is the only one that is different and does not exist in the tutorial. The multirobot group will consist of a 'super' group that contains the 4 robots as subgroups. In order to generate this group click on 'Add Group'. Give the group a name you like, I called it 'multi_arms_group'. Now, click on 'Add Subgroup', next find the Advanced Options section and select the 4 robot planning groups you generated in the previous steps.

You can create home_poses for all the robots if you think you are going to need this in the future. The 'open' and 'close' positions for the grippers are particularly useful to have, given that most grippers in real life are controlled by a simple open/close command. I used the same values suggested in the tutorial. 

At the end you should have something similar to this. Note that all the groups are in the same window, I just scrolled down and took a screenshot. 

![Alt text](media/PlanningGroups1.png?raw=true "CollisionMatrix")
![Alt text](media/PlanningGroups2.png?raw=true "CollisionMatrix")
![Alt text](media/PlanningGroups3.png?raw=true "CollisionMatrix")

## Simulation pane
The simulation step can be skipped in this case, only because the xacro and respective urdf we created are compatible with gazebo. Gazebo will not work if inertia and transmissions for each link and joints have not been defined. The gazebo controller also needs to be included in the xacro file to control the robot in Gazebo. 
In theory, PID values should be also added. However, this process is a trial-error endeavour which we do not have the time for. Gazebo will throw an error when we first launch the simulation and use the PID defualt values, you can ignore this, the default values are good enough for us. 

Now proceed with the author page and generate the config files.

## ROS controller configurations
Usually the ROS controllers are automatically generated by the setup assistant in the ROS Control Pane. This time we are going to do so manually as per the [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/multiple_robot_arms/multiple_robot_arms_tutorial.html). Once again, though, we are going to do something slightly different to allow to control multiple robots at once.

In the following steps we are going to create a total of 5 controllers:
- joint_state_controller
- front_left_arm_controller
- front_right_arm_controller
- rear_left_arm_controller
- rear_right_arm_controller

### joint_state_controller
This contreller reads the joints from the srdf file and publishes the state of ALL joints. Therefore, it does not need changing. Follow the tutorial for this.

### trajectory_controller
This file will be similar to the one in the tutorial but requires the addition of the 2 rear robots (with the respective grippers). Make sure the names of joints and controllers are consistent with the urdf/xacro file. I have also removed the constraints to simplify the inverse kinematics calculations and this tutorial. The file should look like something like this:
<details>
<summary>trajectory_controller.yaml</summary>


```
front_right_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - front_right_arm_joint1
        - front_right_arm_joint2
        - front_right_arm_joint3
        - front_right_arm_joint4
        - front_right_arm_joint5
        - front_right_arm_joint6
        - front_right_arm_joint7
    

front_left_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - front_left_arm_joint1
        - front_left_arm_joint2
        - front_left_arm_joint3
        - front_left_arm_joint4
        - front_left_arm_joint5
        - front_left_arm_joint6
        - front_left_arm_joint7

rear_right_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - rear_right_arm_joint1
        - rear_right_arm_joint2
        - rear_right_arm_joint3
        - rear_right_arm_joint4
        - rear_right_arm_joint5
        - rear_right_arm_joint6
        - rear_right_arm_joint7

rear_left_arm_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
        - rear_left_arm_joint1
        - rear_left_arm_joint2
        - rear_left_arm_joint3
        - rear_left_arm_joint4
        - rear_left_arm_joint5
        - rear_left_arm_joint6
        - rear_left_arm_joint7
    

front_right_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - front_right_arm_finger_joint1
    gains:
        front_right_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}

front_left_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - front_left_arm_finger_joint1
    gains:
        front_left_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}

rear_right_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - rear_right_arm_finger_joint1
    gains:
        rear_right_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}

rear_left_hand_controller:
    type: "effort_controllers/JointTrajectoryController"
    joints:
        - rear_left_arm_finger_joint1
    gains:
        rear_left_arm_finger_joint1:  {p: 50.0, d: 1.0, i: 0.01, i_clamp: 1.0}
```
</details>

### control_utilis.launch
Then, we need to create the `control_utils.launch` as per instructions in the tutorial. This will be used to launch several controllers split into 3 nodes:
- joint state controller - previosuly created, this controller is part of the ros_control family and reads and publishes joint state directly from the robotHardwareInterface (in this case a fake hardware interface given that the joints are provided by Gazebo) _Note this is not to be confused with the joint_state_publisher which is a stand alone node that is not tied with the ros control architecture_
- robot_state_publisher - this node takes joint information (published by the joint_state_controller), computes the forward kinematics and converts them into TF messages hence generating the transforms
- trajectory_controller - this is the yaml file that we previously created which is used to launch the individual robot trajectory controllers and gripper controllers
The launch file should look like similar to this:
```
<?xml version="1.0"?>
<launch>

<!-- Robot state publisher -->
<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="50.0" />
    <param name="tf_prefix" type="string" value="" />
</node>

<!-- Joint state controller -->
<rosparam file="$(find panda_multiple_arms)/config/joint_state_controller.yaml" command="load" />
<node name="joint_state_controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller" respawn="false" output="screen" />

<!-- Joint trajectory controller -->
<rosparam file="$(find panda_multiple_arms)/config/trajectory_controller.yaml" command="load" />
<node name="arms_trajectory_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" 
args="front_right_arm_trajectory_controller front_left_arm_trajectory_controller front_right_hand_controller front_left_hand_controller
rear_right_arm_trajectory_controller rear_left_arm_trajectory_controller rear_right_hand_controller rear_left_hand_controller" 
/>

</launch>
``` 
### moveit autogenerated files
As mentioned before, Moveit automatically generates some launch files and yaml files. The tutorial mentions a coupls of .yaml and .launch files that we need to modify:
- ros_controller.yaml/.launch
- simple_moveit_controller.yaml/.launch
I would not bother with this as these are redunt files which are not used.

### Gazebo integration
The files `panda_multiple_arms_empty_world.launch` and  `bringup_moveit.launch` can be left as the tutorial.

## Demo
If you want to run the demo, open 2 new terminals and copy paste the 2 following commands
`roslaunch panda_multiple_arms bringup_moveit.launch `
`rosrun panda_multiple_arms moveit_controller.py`
