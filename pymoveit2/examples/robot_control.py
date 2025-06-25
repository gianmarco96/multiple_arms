#!/usr/bin/env python3

######################################################################
# Working so far using the namespace argument. I now need to try and call different robots from the same script
# I can try and add each node to a new thread
# Later I need to put the robots close to each other
######################################################################



from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5 as robot
#from pymoveit2.robots import panda as robot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Plane
from geometry_msgs.msg import PoseWithCovarianceStamped
from time import sleep

# for turtlebot
from geometry_msgs.msg import PoseStamped

position = [0.5, 0.01, 0.25]
quat_xyzw = [0.0, 0.0, 0.0, 1.0]
cartesian = False

def main():
    rclpy.init()
    
    node_tb1 = init_robot("tb1", "mobile", [-3.5, -0.5, 0.1])
    node_tb2 = init_robot("tb2", "mobile", [-2.5, -0.5, 0.1])
    node_tb3 = init_robot("tb3", "mobile", [3.5, -0.5, 0.1])
    node_tb4 = init_robot("tb4", "mobile", [-1.5, -0.5, 0.1])

    node_arm1, arm1 = init_robot("arm1", "arm", []) 

    # # # Create second node to control arm2
    node_arm2, arm2= init_robot("arm2", "arm", [])

    # node_arm3, arm3 = init_robot("arm3", "arm") 

    # # Create second node to control arm2
    # node_arm4, arm4= init_robot("arm4", "arm")
    
    
    
    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(9)

    executor.add_node(node_arm1)
    executor.add_node(node_arm2)
    # executor.add_node(node_arm3)
    # executor.add_node(node_arm4)
    
    executor.add_node(node_tb1)
    executor.add_node(node_tb2)
    executor.add_node(node_tb3)
    executor.add_node(node_tb4)



    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    add_ground_plane(node_arm1)
    add_ground_plane(node_arm2)
    # add_ground_plane(node_arm3)
    # add_ground_plane(node_arm4)

    x_y_pos = [0.3,  0.10]

    try:
        robot_move(arm1, node_arm1, x_y_pos)
        robot_move(arm2, node_arm2, x_y_pos)
        sleep(2)
        # robot_move(arm3, node_arm3, x_y_pos)
        # robot_move(arm4, node_arm4, x_y_pos)


        robot_move(None, node_tb1, [3.0, -0.5])
        robot_move(None, node_tb2, [3.0, 0.5])
        sleep(2)
        robot_move(None, node_tb3, [3.0, -0.5])
        robot_move(None, node_tb4, [3.0, 0.5])
        # print("I'm here")
        # node_tb1.publish_pose()
    except Exception as err:
        node_arm1.get_logger().info(f'Exception occured. {err}')
        node_arm2.get_logger().info(f'Exception occured. {err}')
        # node_arm3.get_logger().info(f'Exception occured. {err}')
        # node_arm4.get_logger().info(f'Exception occured. {err}')

        node_tb1.get_logger().info(f'Exception occured. {err}')
        node_tb2.get_logger().info(f'Exception occured. {err}')
        node_tb3.get_logger().info(f'Exception occured. {err}')
        node_tb4.get_logger().info(f'Exception occured. {err}')

    sleep(5)
    rclpy.shutdown()
    exit(0)


def add_ground_plane(node):

    # Create a CollisionObject message
    collision_object = CollisionObject()
    collision_object.id = "ground_plane"
    collision_object.header.frame_id = "world"

    # Define the ground plane as a box shape
    ground_plane = Plane()
    ground_plane.coef = [0.0, 0.0, 1.0, 0.0]

    # Set the ground plane's pose
    ground_plane_pose = Pose()
    ground_plane_pose.position.z = -0.005  # Adjust the height of the ground plane

    collision_object.planes.append(ground_plane)
    collision_object.plane_poses.append(ground_plane_pose)

    # Create a PlanningScene message
    scene = PlanningScene()
    scene.world.collision_objects.append(collision_object)
    scene.is_diff = True
    
    publisher_ = node.create_publisher(PlanningScene, 'planning_scene', 10)
    publisher_.publish(scene)

def init_robot(_namespace, type, initial_pose):
    if type == "arm":
        # Create first node to control arm1
        node = Node("robot_controller", namespace=_namespace)

        node.get_logger().info(f'Starting node')


        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        moveit2 = MoveIt2(
            node=node,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group
        )
        return node, moveit2
    
    if type == "mobile":
        node = mobile_robot(_namespace)
        node.publish_pose_with_co(initial_pose)
        return node


def robot_move(moveit2, node, x_y_pos):
    if moveit2 == None:
        node.publish_pose(x_y_pos)
        node.get_logger().info(f'Mobile robot pose goal sent to robot')
    else:
        position[0] = x_y_pos[0]
        position[1] = x_y_pos[1]
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        node.get_logger().info(f'Arm pose goal sent to robot')

class mobile_robot(Node):
    def __init__(self, namespace):
        super().__init__(namespace)
        self.topic = namespace + '/goal_pose'
        self.initial_pose_topic = namespace + '/initialpose'
        self.publisher = self.create_publisher(PoseStamped, self.topic, 10)
        self.publisherWithCo = self.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)

    def publish_pose(self, x_y_pos):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x_y_pos[0]
        pose_msg.pose.position.y = x_y_pos[1]
        pose_msg.pose.orientation.w = 1.0
        self.publisher.publish(pose_msg)
        sleep(0.3)

    def publish_pose_with_co(self, initial_pose):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = initial_pose[0]
        pose_msg.pose.pose.position.y = initial_pose[1]
        pose_msg.pose.pose.position.y = initial_pose[2]
        pose_msg.pose.pose.orientation.w = 1.0
        self.publisherWithCo.publish(pose_msg)
        sleep(0.1)



if __name__ == "__main__":
    main()