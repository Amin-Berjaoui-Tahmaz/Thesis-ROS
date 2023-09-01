#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np

class ObservationNode:
    def __init__(self):
        rospy.init_node('observation_node')

        self.env = 'cleanup'

        self.cartesian_pose = None
        self.dimension_object1 = None
        self.dimension_object2 = None
        self.gripper_states = None

        self.table_center_robot = 0.51
        self.table_height = 0.8
        self.dist_camera_table_center = 0.58 #measured

        rospy.Subscriber('/cartesian_pose', PoseStamped, self.cartesian_pose_callback)
        rospy.Subscriber('/dope/pose_meat', PoseStamped, self.dimension_object1_callback)
        rospy.Subscriber('/dope/pose_gelatin', PoseStamped, self.dimension_object2_callback)
        rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_callback)

        self.combined_obs_pub = rospy.Publisher('/observation', Float64MultiArray, queue_size=1)

    def cartesian_pose_callback(self, data):
        self.cartesian_pose = data
#        self.combine_and_publish()

    def dimension_object1_callback(self, data):
        self.dimension_object1 = data
#        self.combine_and_publish()

    def dimension_object2_callback(self, data):
        self.dimension_object2 = data
#        self.combine_and_publish()

    def gripper_callback(self, data):
        self.gripper_states = data
#        self.combine_and_publish()

    def extract_cartesian_values(self, pose_stamped_msg):
        x = pose_stamped_msg.pose.position.x
        y = pose_stamped_msg.pose.position.y
        z = pose_stamped_msg.pose.position.z

        quat1 = pose_stamped_msg.pose.orientation.x
        quat2 = pose_stamped_msg.pose.orientation.y
        quat3 = pose_stamped_msg.pose.orientation.z
        quat4 = pose_stamped_msg.pose.orientation.w

        x = x - self.table_center_robot
        z = z + self.table_height

        return [x, y, z, quat1, quat2, quat3, quat4]

    def extract_object_pose(self, object_pose_msg):

        x_obj_measured = object_pose_msg.pose.position.x
        # y_obj_measured = object_pose_msg.pose.position.y
        z_obj_measured = object_pose_msg.pose.position.z

        # x_obj, y_obj, z_obj = 0,0,0.83

        x_obj = self.dist_camera_table_center - z_obj_measured
        y_obj = x_obj_measured
        z_obj = self.table_height + 0.03

        # print('self.dist_camera_table_center',self.dist_camera_table_center)
        # print('self.table_center_robot',self.table_center_robot)
        # print('dist_obj_table_center',dist_obj_table_center)

        # quat1_obj = object_pose_msg.pose.orientation.x    
        # quat2_obj = object_pose_msg.pose.orientation.y
        quat3_obj = object_pose_msg.pose.orientation.z
        quat4_obj = object_pose_msg.pose.orientation.w

        quat1_obj, quat2_obj, quat3_obj, quat4_obj = [0.,          0.,          quat3_obj, quat4_obj]

        return [x_obj, y_obj, z_obj, quat1_obj, quat2_obj, quat3_obj, quat4_obj]

    def extract_gripper_pos_vel(self, joint_state_msg):
        
        pos_finger1 = joint_state_msg.position[0]/2
        pos_finger2 = -1 * pos_finger1

        vel_finger1 = joint_state_msg.velocity[0]
        vel_finger2 = vel_finger1

        return [pos_finger1, pos_finger2, vel_finger1, vel_finger2]

    def combine_and_publish(self):
        """
        Gets the observation in the form:
            [eef_xyz, eef_quat, gripper_pos, gripper_vel, object_pos, object_quat, gripper_to_cube]
        """

        if self.env=='lift':
            print(self.dimension_object1)
            if self.cartesian_pose is not None and self.dimension_object1 is not None:         # Only publish when both pieces of information are available
                cartesian_values = self.extract_cartesian_values(self.cartesian_pose)
                gripper_values = self.extract_gripper_pos_vel(self.gripper_states)

                object1_state_values = self.extract_object_pose(self.dimension_object1)
                object1_state_pos = object1_state_values[:3]
                object1_state_quat = object1_state_values[3:]

                gripper_to_cube = [c_value - o_value for c_value, o_value in zip(cartesian_values[:3], object1_state_values[:3])]
                print('Transformed Cartesian Pose', np.round(cartesian_values[:3],4))
                print('object_state_values',np.round(object1_state_values[:3],3))
                # print('gripper_to_cube',np.round(gripper_to_cube,3))
                # print('gripper values',np.round(gripper_values,4))
                print('#######')

                combined_data = cartesian_values + gripper_values + object1_state_pos + object1_state_quat + gripper_to_cube

                action_msg = Float64MultiArray(data=combined_data)
                self.combined_obs_pub.publish(action_msg)

                # Introduce a 4-second break
                rospy.sleep(4)  # 4 seconds

        elif self.env=='cleanup':
            if self.cartesian_pose is not None and self.dimension_object1 and self.dimension_object2 is not None:         # Only publish when both pieces of information are available
                cartesian_values = self.extract_cartesian_values(self.cartesian_pose)
                gripper_values = self.extract_gripper_pos_vel(self.gripper_states)
                object1_state_values = self.extract_object_pose(self.dimension_object1)
                object2_state_values = self.extract_object_pose(self.dimension_object2)

                object1_state_pos = object1_state_values[:3]
                object1_state_quat = object1_state_values[3:]

                object2_state_pos = object2_state_values[:3]
                object2_state_quat = object2_state_values[3:]

                gripper_to_cube = [c_value - o_value for c_value, o_value in zip(cartesian_values[:3], object1_state_values[:3])]
                print('Transformed Cartesian Pose', np.round(cartesian_values[:3],4))
                print('object1_state_values',np.round(object1_state_values[:3],3))
                print('object2_state_values',np.round(object2_state_values[:3],3))
                print('#######')

                combined_data = cartesian_values + gripper_values + object1_state_pos + object2_state_pos + object1_state_quat + object2_state_quat

                action_msg = Float64MultiArray(data=combined_data)
                self.combined_obs_pub.publish(action_msg)

                # Introduce a 4-second break
                rospy.sleep(4)  # 4 seconds

        # else:
        #     combined_data = [-0.1062621,   0.01249285,  1.0105337,   0.9982049,  -0.00298124,  0.05947854, -0.00635711,  0.020833,   -0.020833,    0.,          0.,         -0.05833616, 0.01129947,  0.83,        0.,          0.,          0.5968902,   0.8023229, -0.04792594,  0.00119338,  0.1805337 ]
        #     print('floop')

        #     action_msg = Float64MultiArray(data=combined_data)
        #     self.combined_obs_pub.publish(action_msg)

        #     # Introduce a 4-second break
        #     rospy.sleep(4)  # 4 seconds

    def run(self):
        #rospy.spin()
        rate = rospy.Rate(1)  # Adjust the rate as needed (1 Hz in this example)
        while not rospy.is_shutdown():
            if self.cartesian_pose is not None or self.dimension_object1 is not None:
                self.combine_and_publish()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = ObservationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass