#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String

# Initialise the node and Moveit!
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("multiarm_commanderV2", anonymous=True)

# Notice that we are using the multi_arms_group. This has access to all the individual robot subgroups
group_name = "multi_arms_group"
move_group = moveit_commander.MoveGroupCommander(group_name)

robot = moveit_commander.RobotCommander()

# Here we are getting the groups joints. Notice how this prints out the joint of all the 4 robots
joint_goal = move_group.get_current_joint_values()
print("###################################################################")
rospy.loginfo(joint_goal)
print("###################################################################")

# This is how you can access the specific robot groups
front_left_arm = robot.get_group("front_left_arm")
front_right_arm = robot.get_group("front_right_arm")
rear_left_arm = robot.get_group("rear_left_arm")
rear_right_arm = robot.get_group("rear_right_arm")

# And here you can print each individual robot joint state
front_left_arm_joints = front_left_arm.get_current_joint_values()
front_right_arm_joints = front_right_arm.get_current_joint_values()
rear_left_arm_joints = rear_left_arm.get_current_joint_values()
rear_right_arm_joints = rear_right_arm.get_current_joint_values()
print("###################################################################")
rospy.loginfo("Front left robot joint state:")
rospy.loginfo(front_left_arm_joints)
rospy.loginfo("Front right robot joint state:")
rospy.loginfo(front_right_arm_joints)
rospy.loginfo("Rear left robot joint state:")
rospy.loginfo(rear_left_arm_joints)
rospy.loginfo("Rear right robot joint state:")
rospy.loginfo(rear_right_arm_joints)
print("###################################################################")

# Next I am going to execute a joint motion. I want the robot to execute the same motion
# so I will just use one of the robot joints to create the goal which will be the same for all 4 
front_left_arm_joints[0] = 0
front_left_arm_joints[1] = math.radians(1)
front_left_arm_joints[2] = 0
front_left_arm_joints[3] = math.radians(-118)
front_left_arm_joints[4] = 0
front_left_arm_joints[5] = math.radians(119)
front_left_arm_joints[6] = math.radians(0)

joint_goal = front_left_arm_joints + front_left_arm_joints + front_left_arm_joints + front_left_arm_joints

print("###################################################################")
rospy.loginfo("Executing joint goal")
move_group.plan(joint_goal)
move_group.go(wait=True)
move_group.stop()

# The next section of the code aims to show how you can control any number of robots at one time
# I will execute a few pose goals only for the front 2 robots

# This is how you can access the individual robot pose. Notice the end_effector_link argument.
# This argument is often omitted when using a single robot planning group but becomes crucial 
# when multiple robots are controlled at once
print("###################################################################")
rospy.loginfo("Printing front left arm pose")
rospy.loginfo(move_group.get_current_pose(end_effector_link="front_left_arm_link8"))
print("###################################################################")

# Constructing the pose goal
front_left_arm_pose = geometry_msgs.msg.Pose()
front_right_arm_pose = geometry_msgs.msg.Pose()

# Changing the orientation of the robot by setting the orientation w to 1. For more information 
# check ROS quaternion
front_left_arm_pose.orientation.w = 1.0
front_left_arm_pose.position.x = 1.05
front_left_arm_pose.position.y = 0.5
front_left_arm_pose.position.z = 1.65

front_right_arm_pose.orientation.w = 1.0
front_right_arm_pose.position.x = 1.05
front_right_arm_pose.position.y = -0.5
front_right_arm_pose.position.z = 1.65

move_group.set_pose_target(front_left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(front_right_arm_pose,end_effector_link="front_right_arm_link8")
move_group.go(wait=True)
move_group.stop()

# Motion 2
front_left_arm_pose.orientation.w = 1.0
front_left_arm_pose.position.x = 1.05
front_left_arm_pose.position.y = 0.5
front_left_arm_pose.position.z = 1.75

front_right_arm_pose.orientation.w = 1.0
front_right_arm_pose.position.x = 1.05
front_right_arm_pose.position.y = -0.5
front_right_arm_pose.position.z = 1.75

move_group.set_pose_target(front_left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(front_right_arm_pose,end_effector_link="front_right_arm_link8")
success = move_group.go(wait=True)
move_group.stop()

# Motion 3
front_left_arm_pose.orientation.w = 1.0
front_left_arm_pose.position.x = 1.05
front_left_arm_pose.position.y = 0.5
front_left_arm_pose.position.z = 1.65

front_right_arm_pose.orientation.w = 1.0
front_right_arm_pose.position.x = 1.05
front_right_arm_pose.position.y = -0.5
front_right_arm_pose.position.z = 1.65

move_group.set_pose_target(front_left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(front_right_arm_pose,end_effector_link="front_right_arm_link8")
success = move_group.go(wait=True)
move_group.stop()
