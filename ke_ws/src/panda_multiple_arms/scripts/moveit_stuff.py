#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
   

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "both_arms"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())
    print(group.get_current_joint_values())
    left = robot.get_group("left_arm")
    right = robot.get_group("right_arm")
    left_joint = left.get_current_joint_values()
    right_joint = right.get_current_joint_values()
    right_joint[0] = 0.785
    left_joint[0] = 0.785 
    #group.go([left_joint, right_joint], wait=True)

if __name__ == "__main__":
    main()
