#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray
import maple.torch.pytorch_util as ptu
import numpy as np

import torch
import torch.nn as nn

class PyTorchModelNode:
    def __init__(self):

        rospy.init_node('pytorch_model_node')
        self.previous_action = None  # Initialize previous_action to None
        self.action_tolerance = 1e-3  # Set a tolerance value for action comparison

#        dir = '/home/amin/Desktop/maple_baselines/lift/05-16-test/05-16-test_2023_05_16_05_46_39_0000--s-64061/itr_800.pkl'
        dir = '/home/amin/Desktop/maple_baselines/cleanup/05-16-test/05-16-test_2023_05_16_05_32_07_0000--s-2471/itr_725.pkl'
#        dir = '/home/amin/Desktop/maple/data/cleanup_mod_env/08-24-512-newaff-push/08-24-512-newaff-push_2023_08_24_00_28_22_0000--s-84482/itr_240.pkl'

        self.global_xyz_bounds = np.array([[-0.30, -0.30, 0.80],[0.15, 0.30, 0.85]])
        
        #### NEED TO SET GLOBAL_XYZ_BOUNDS AND UNNORMALIZATIONS FOR EACH PRIMITIVE DEPENDING ON THE ENVIRONMENT ####

        self.model = (torch.load(dir, map_location='cuda:0') if ptu.gpu_enabled() else torch.load(dir, map_location='cpu'))
        self.model = self.model["evaluation/policy"].eval()

        rospy.Subscriber('/observation', Float64MultiArray, self.observation_callback)
        self.action_pub = rospy.Publisher('/action', Float64MultiArray, queue_size=1)

        self.reach_pub = rospy.Publisher('/reach', Float64MultiArray, queue_size=1)
        self.push_pub = rospy.Publisher('/push', Float64MultiArray, queue_size=1)
        self.grasp_pub = rospy.Publisher('/grasp', Float64MultiArray, queue_size=1)
        self.stiffness_pub = rospy.Publisher('/stiffness_action', Float32MultiArray, queue_size=1)

    def are_actions_equal(self, action1, action2):
        return np.all(np.abs(action1 - action2) < self.action_tolerance)

    def _get_unnormalized_pos(self, pos, bounds):
        pos = np.clip(pos, -1, 1)
        pos = (pos + 1) / 2
        low, high = bounds[0], bounds[1]
        return low + (high - low) * pos

    def observation_callback(self, data):

        obs_array = np.array(data.data, dtype=np.float32)
        obs_tensor = torch.from_numpy(obs_array).reshape(1,-1)  # Reshape the tensor to desired shape

        action = self.model.get_actions(obs_tensor)

        num_skills = 5
        skills = ['atomic', 'reach', 'grasp', 'push', 'open']

        normalized_pos = np.array(action[0][num_skills:num_skills+3])
        unnormalized_pos = self._get_unnormalized_pos(
            normalized_pos, self.global_xyz_bounds)

        # Adjusts according to the lab measurements and simulation assumptions (such as table height, and table center is co-ordinate frame)
        unnormalized_pos[0] = unnormalized_pos[0] + 0.55
        unnormalized_pos[2] = unnormalized_pos[2] - 0.8

        action[0][num_skills:num_skills+3] = unnormalized_pos

        if self.previous_action is not None and self.are_actions_equal(action, self.previous_action):         # Check if the new action is the same as the previous action
            print("No new action.")
            print('################################')
        else:
            primitive = np.round(action[0][:num_skills])
            # Extract the primitive so you publish to the right topic

            skill_index = np.where(primitive==1)
            current_skill = skills[skill_index[0][0]]
            primitive_params = np.round(action[0][num_skills:],3)

            print("Primitive Name:", current_skill)
            print('Primitive Type', primitive)
            print('Action', primitive_params)
            print('Unnormalized parameters', unnormalized_pos)

            action_msg = Float64MultiArray(data=primitive_params.flatten().tolist())  # Convert NumPy array to list

            if current_skill=='reach':
                self.reach_pub.publish(action_msg)
            if current_skill=='push':
                push_temp = action_msg.data[-3:]
                push_temp = np.clip(push_temp, -1, 1)
                push_temp *= np.array([0.25, 0.25, 0.05])
                print('Delta Unnormalized',push_temp)
                self.push_pub.publish(action_msg)
            if current_skill=='grasp':
                self.grasp_pub.publish(action_msg)

            self.action_pub.publish(action_msg)
            stiffness_msg = Float32MultiArray()
            stiffness_msg.data = [100, 100, 100] #action_msg.data[:3]
            self.stiffness_pub.publish(stiffness_msg)

        self.previous_action = np.round(action,4)  # Update previous_action
        print('################################')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PyTorchModelNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
