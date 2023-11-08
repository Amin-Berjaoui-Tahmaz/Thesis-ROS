#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import MoveActionGoal
import numpy as np

class EquilibriumPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('equilibrium_publisher_node')

        # Subscribe to the three Float64MultiArray topics
        rospy.Subscriber('/reach', Float64MultiArray, self.reach_callback)
        rospy.Subscriber('/push', Float64MultiArray, self.push_callback)
        rospy.Subscriber('/grasp', Float64MultiArray, self.grasp_callback)
        rospy.Subscriber('/open', Float64MultiArray, self.open_callback)

        rospy.Subscriber('/stiffness_action', Float32MultiArray, self.stiffness_callback)
        rospy.Subscriber('/cartesian_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/franka_gripper/joint_states', PoseStamped, self.gripper_callback)

        # Create a publisher for the PoseStamped topic
        self.equilibrium_pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=1)
        self.action_complete_pub = rospy.Publisher('/action_complete', Bool, queue_size=10)

        self.delta_xyz_scale = [0.25, 0.25, 0.05]
        self.lift_height = 0.2

        self.current_pose = None
        self.reach_values = None
        self.gripper_state = None
        self.stiffness = None

        # Create a message to control gripper
        move_goal_msg = MoveActionGoal()
        move_goal_msg.goal.width = 0.1  # Set the desired gripper width
        move_goal_msg.goal.speed = 0.1   # Set the desired gripper speed
        self.gripper_pub.publish(move_goal_msg)

    def publish_action_complete(self,set_bool):
        action_complete_msg = Bool()
        action_complete_msg.data = set_bool
        self.action_complete_pub.publish(action_complete_msg)

    def gripper_callback(self, msg):
        print('gripper position' ,msg.position)
        print('gripper position rounded' , round(msg.position,4))
        print('if the value is 0 then just to bool reflecting the current state of gripper (open/closed)')
        self.gripper_state = bool(round(msg.position,4))

    def stiffness_callback(self, msg):
        self.stiffness = msg.data

    def pose_callback(self, msg):
        self.current_pose = msg

    def reach_callback(self, data):

        mutex = threading.Lock()
        mutex.acquire()

        self.reach_values = data.data

        # Create a PoseStamped message and populate it with data from the /grasp topic
        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = self.current_pose.header.frame_id

        new_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        new_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        new_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        new_pose.pose.orientation.w = self.current_pose.pose.orientation.w

        new_pose.pose.orientation.x =  0.999 #amin
        new_pose.pose.orientation.y =  0.000
        new_pose.pose.orientation.z =  0.000
        new_pose.pose.orientation.w =  0.000

        new_pose.pose.position.x = self.current_pose.pose.position.x
        new_pose.pose.position.y = self.current_pose.pose.position.y
        new_pose.pose.position.z = self.lift_height

        print('POS 1')
        print(new_pose.pose.position)

        input("Press Enter to publish the received message to /equilibrium_pose...")
        self.equilibrium_pose_pub.publish(new_pose)
        rospy.sleep(2.0)         # Wait for 2 seconds

        # Second, go to the desired x and y eef positions
        new_pose.pose.position.x = self.reach_values[0] + 0.04
        new_pose.pose.position.y = self.reach_values[1]

        new_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        new_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        new_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        new_pose.pose.orientation.w = self.current_pose.pose.orientation.w

        new_pose.pose.orientation.x =  0.999
        new_pose.pose.orientation.y =  0.000
        new_pose.pose.orientation.z =  0.000
        new_pose.pose.orientation.w =  0.000 #amin

        print('POS 2')
        print(new_pose.pose.position)

        input("Press Enter to publish this pose to /equilibrium_pose...")
        self.equilibrium_pose_pub.publish(new_pose)
        rospy.sleep(2.0)         # Wait for 2 seconds

        # Third, go to the desired height
        new_pose.pose.position.z = self.reach_values[2] #-0.02 #+ 0.01 #- 0.02 #+0.04#- 0.04 #+ 0.04

        new_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        new_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        new_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        new_pose.pose.orientation.w = self.current_pose.pose.orientation.w

        new_pose.pose.orientation.x =  0.999
        new_pose.pose.orientation.y =  0.000
        new_pose.pose.orientation.z =  0.000
        new_pose.pose.orientation.w =  0.000 #amin

        print('POS 3')
        print(new_pose.pose.position)

        input("Press Enter to publish the received message to /equilibrium_pose...")
        self.equilibrium_pose_pub.publish(new_pose)
        rospy.sleep(2.0)         # Wait for 2 seconds

        self.reach_final_pose = new_pose

        mutex.release()

        print('################################')

    def push_callback(self, data):

        mutex = threading.Lock()
        mutex.acquire()

        print('Current Stiffness', self.stiffness)
        print('Recieved Push Parameters', data.data)

        input("Press Enter to HALF-CLOSE GRIPPER...")
        # Create a message to control gripper
        move_goal_msg = MoveActionGoal()
        move_goal_msg.goal.width = 0.05 #0.0  # Set the desired gripper width
        move_goal_msg.goal.speed = 0.1   # Set the desired gripper speed
        self.gripper_pub.publish(move_goal_msg)

        self.reach_callback(data)

        # Varies if fixed/variable_kp_mod.
        # In case of fixed:
        
        push_values = data.data[-3:]

        push_values = np.clip(push_values, -1, 1)
        push_values *= self.delta_xyz_scale

        print('reach_final_pose', self.reach_final_pose.pose.position)
        print('push values',push_values)

        push_pose = PoseStamped()
        push_pose.header.stamp = rospy.Time.now()
        push_pose.header.frame_id = self.current_pose.header.frame_id

        push_pose.pose.position.x = self.reach_final_pose.pose.position.x + push_values[0]
        push_pose.pose.position.y = self.reach_final_pose.pose.position.y + push_values[1]
        push_pose.pose.position.z = self.reach_final_pose.pose.position.z + push_values[2] - 0.04

        # push_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        # push_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        # push_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        # push_pose.pose.orientation.w = self.current_pose.pose.orientation.w

        push_pose.pose.orientation.x =  0.999
        push_pose.pose.orientation.y =  0.000
        push_pose.pose.orientation.z =  0.000
        push_pose.pose.orientation.w =  0.000 #amin

        print('POS 4')
        print(push_pose.pose.position)

        input("Press Enter to publish the received message to /equilibrium_pose...")
        self.equilibrium_pose_pub.publish(push_pose)

        rospy.sleep(2.0)         # Wait for 2 seconds
        self.publish_action_complete(True)

        # Create a message to control gripper
        move_goal_msg = MoveActionGoal()
        move_goal_msg.goal.width = 1 #0.0  # Set the desired gripper width
        move_goal_msg.goal.speed = 1   # Set the desired gripper speed

        push_pose.pose.position.x = 0.36
        push_pose.pose.position.y = 0
        push_pose.pose.position.z = 0.2

        push_pose.pose.orientation.x =  0.999
        push_pose.pose.orientation.y =  0.000
        push_pose.pose.orientation.z =  0.000
        push_pose.pose.orientation.w =  0.000 #amin

        input("Press Enter to RESET...")
        self.gripper_pub.publish(move_goal_msg)
        self.equilibrium_pose_pub.publish(push_pose)


        mutex.release()

        print('################################')

    def grasp_callback(self, data):

        mutex = threading.Lock()
        mutex.acquire()

        print('Current Stiffness', self.stiffness)

#        rospy.sleep(2.0)         # Wait for 2 seconds
#        self.gripper_pub.publish(move_goal_msg)

        self.reach_callback(data)

 #       rospy.sleep(2.0)         # Wait for 2 seconds

        # Create a message to control gripper
        move_goal_msg = MoveActionGoal()

        # Fill in the message with the desired data
        print('Gripper Closing!')
        move_goal_msg.goal.width = 0.3#3  # Set the desired gripper width
        move_goal_msg.goal.speed = 1   # Set the desired gripper speed

        input("Press Enter to CLOSE GRIPPER...")
        self.gripper_pub.publish(move_goal_msg)
        rospy.sleep(2.0)         # Wait for 2 seconds

        self.publish_action_complete(True)

        mutex.release()

        print('################################')


    def open_callback(self, data):

        # Create a message to control gripper
        move_goal_msg = MoveActionGoal()

        # Fill in the message with the desired data
        move_goal_msg.goal.width = 1  # Set the desired gripper width
        move_goal_msg.goal.speed = 1   # Set the desired gripper speed

        print('Gripper Opening!')
        input("Press Enter to publish the received message to /equilibrium_pose...")
        self.gripper_pub.publish(move_goal_msg)
        rospy.sleep(2.0)         # Wait for 2 seconds

        self.publish_action_complete(True)

        print('################################')


if __name__ == '__main__':
    try:
        equilibrium_publisher = EquilibriumPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
