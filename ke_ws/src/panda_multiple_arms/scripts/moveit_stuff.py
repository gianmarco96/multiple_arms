#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("multiarm_commanderV2", anonymous=True)

group_name = "multi_arms_group"
move_group = moveit_commander.MoveGroupCommander(group_name)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

joint_goal = move_group.get_current_joint_values()
# cr_pose = move_group.get_current_pose()
# print(joint_goal)
# print(cr_pose,"right_hand")


left_arm = robot.get_group("front_left_arm")
right_arm = robot.get_group("front_right_arm")

left_arm_joints = left_arm.get_current_joint_values()
right_arm_joints = right_arm.get_current_joint_values()

left_arm_joints[0] = 0
left_arm_joints[1] = math.radians(1)
left_arm_joints[2] = 0
left_arm_joints[3] = math.radians(-118)
left_arm_joints[4] = 0
left_arm_joints[5] = math.radians(119)
left_arm_joints[6] = math.radians(0)

right_arm_joints[0] = 0
right_arm_joints[1] = math.radians(1)
right_arm_joints[2] = 0
right_arm_joints[3] = math.radians(-118)
right_arm_joints[4] = 0
right_arm_joints[5] = math.radians(119)
right_arm_joints[6] = math.radians(0)

joint_goal = left_arm_joints + right_arm_joints + left_arm_joints + left_arm_joints
print("#########################################")
print("Executing Joint Command!")
print("#########################################")
move_group.plan(joint_goal)
move_group.go(wait=True)
move_group.stop()

left_arm_pose = geometry_msgs.msg.Pose()
right_arm_pose = geometry_msgs.msg.Pose()

left_arm_pose.orientation.w = 1.0
left_arm_pose.position.x = 1.05
left_arm_pose.position.y = 0.5
left_arm_pose.position.z = 1.65

right_arm_pose.orientation.w = 1.0
right_arm_pose.position.x = 1.05
right_arm_pose.position.y = -0.5
right_arm_pose.position.z = 1.65

print(left_arm.get_current_pose())
print(right_arm.get_current_pose())


print("end e =")
print(move_group.get_current_pose(end_effector_link="front_left_arm_link8"))

move_group.set_pose_target(left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(right_arm_pose,end_effector_link="front_right_arm_link8")

success = move_group.go(wait=True)
move_group.stop()

# Motion 2
left_arm_pose.orientation.w = 1.0
left_arm_pose.position.x = 1.05
left_arm_pose.position.y = 0.5
left_arm_pose.position.z = 1.75

right_arm_pose.orientation.w = 1.0
right_arm_pose.position.x = 1.05
right_arm_pose.position.y = -0.5
right_arm_pose.position.z = 1.75

move_group.set_pose_target(left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(right_arm_pose,end_effector_link="front_right_arm_link8")
success = move_group.go(wait=True)
move_group.stop()

# Motion 3

left_arm_pose.orientation.w = 1.0
left_arm_pose.position.x = 1.05
left_arm_pose.position.y = 0.5
left_arm_pose.position.z = 1.65

right_arm_pose.orientation.w = 1.0
right_arm_pose.position.x = 1.05
right_arm_pose.position.y = -0.5
right_arm_pose.position.z = 1.65

move_group.set_pose_target(left_arm_pose,end_effector_link="front_left_arm_link8")
move_group.set_pose_target(right_arm_pose,end_effector_link="front_right_arm_link8")
success = move_group.go(wait=True)
move_group.stop()
