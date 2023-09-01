#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import maple.torch.pytorch_util as ptu
import numpy as np

import torch
import torch.nn as nn

class PyTorchModelNode:
    def __init__(self):

        rospy.init_node('pytorch_model_node')
        self.previous_action = None  # Initialize previous_action to None
        self.action_tolerance = 1e-3  # Set a tolerance value for action comparison

        dir = '/home/amin/Desktop/maple_baselines/lift/05-16-test/05-16-test_2023_05_16_05_46_39_0000--s-64061/itr_800.pkl'

#        dir = '/home/amin/Desktop/maple_baselines/cleanup/05-16-test/05-16-test_2023_05_16_05_32_07_0000--s-2471/itr_725.pkl'

        self.global_xyz_bounds = np.array([[-0.30, -0.30, 0.80],[0.15, 0.30, 0.85]])

        #### NEED TO SET GLOBAL_XYZ_BOUNDS AND UNNORMALIZATIONS FOR EACH PRIMITIVE DEPENDING ON THE ENVIRONMENT ####

        self.model = (torch.load(dir, map_location='cuda:0') if ptu.gpu_enabled() else torch.load(dir, map_location='cpu'))
        self.model = self.model["evaluation/policy"].eval()

        rospy.Subscriber('/observation', Float64MultiArray, self.observation_callback)
        self.action_pub = rospy.Publisher('/action', Float64MultiArray, queue_size=1)

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
        normalized_pos = np.array(action[0][num_skills:num_skills+3])
        unnormalized_pos = self._get_unnormalized_pos(
            normalized_pos, self.global_xyz_bounds)
        action[0][num_skills:num_skills+3] = unnormalized_pos

        if self.previous_action is not None and self.are_actions_equal(action, self.previous_action):         # Check if the new action is the same as the previous action
            print("No new action.")
            print('################################')
        else:
            print('primitive', np.round(action[0][:num_skills]))
            print('action', np.round(action[0][num_skills:],3))
#            print('unnormalized action', unnormalized_pos)
            print('################################')

            action_msg = Float64MultiArray(data=action.flatten().tolist())  # Convert NumPy array to list
            self.action_pub.publish(action_msg)

        self.previous_action = np.round(action,5)  # Update previous_action

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PyTorchModelNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
