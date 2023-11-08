#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_msgs.msg import String
import maple.torch.pytorch_util as ptu
import numpy as np

import torch
import torch.nn as nn

class PyTorchModelNode:
    def __init__(self):

        rospy.init_node('pytorch_model_node')
        self.previous_action = None  # Initialize previous_action to None
        self.action_tolerance = 1e-3  # Set a tolerance value for action comparison
        self.policy_initialized = False

        self.stiffness_mode = True
        self.stiffness_lower_bound, self.stiffness_upper_bound = [50,400]

        self.global_xyz_bounds = np.array([[-0.30, -0.30, 0.80],[0.15, 0.30, 0.85]])
        
        #### NEED TO SET GLOBAL_XYZ_BOUNDS AND UNNORMALIZATIONS FOR EACH PRIMITIVE DEPENDING ON THE ENVIRONMENT ####

        rospy.Subscriber('/observation', Float64MultiArray, self.observation_callback)
        rospy.Subscriber('/policy', String, self.policy_callback)

        self.action_pub = rospy.Publisher('/action', Float64MultiArray, queue_size=1)
        self.stiffness_pub = rospy.Publisher('/stiffness_action', Float32MultiArray, queue_size=1)

        self.reach_pub = rospy.Publisher('/reach', Float64MultiArray, queue_size=1)
        self.push_pub = rospy.Publisher('/push', Float64MultiArray, queue_size=1)
        self.grasp_pub = rospy.Publisher('/grasp', Float64MultiArray, queue_size=1)
        self.open_pub = rospy.Publisher('/open', Float64MultiArray, queue_size=1)

    def policy_callback(self,data):

        self.policy = str(data.data)

        # Split the variable into parts using '/' as the delimiter
        parts = self.policy.split('/')

        # Extract "cleanup" and "push" if they exist
        if "cleanup" in parts and "push" in parts:
            env = 'cleanup_push'
        elif "cleanup" in parts and "pnp" in parts:
            env = 'cleanup_pnp'
        elif "lift" in parts:
            env = 'lift'

        print("Mode", env)

        if env == "lift":
            self.skills = ['reach', 'grasp']
        elif env == "cleanup_push":
            self.skills = ['push']
        elif env == "cleanup_pnp":
            self.skills = ['reach', 'grasp','open']



#        self.skills = ['atomic', 'reach', 'grasp', 'push', 'open']
#        self.skills = ['reach', 'grasp']
#        self.skills = ['reach', 'grasp','open']
#        self.skills = ['push']
        self.num_skills = len(self.skills)

        if self.num_skills==1:
           self.num_skills = 0

        self.model = (torch.load(self.policy, map_location='cuda:0') if ptu.gpu_enabled() else torch.load(self.policy, map_location='cpu'))
        self.model = self.model["evaluation/policy"].eval()

        self.policy_initialized = True

    def are_actions_equal(self, action1, action2):
        return np.all(np.abs(action1 - action2) < self.action_tolerance)

    def _get_unnormalized_pos(self, pos, bounds):
        pos = np.clip(pos, -1, 1)
        pos = (pos + 1) / 2
        low, high = bounds[0], bounds[1]
        return low + (high - low) * pos

    def observation_callback(self, data):

        if self.policy_initialized:
            obs_array = np.array(data.data, dtype=np.float32)
            obs_tensor = torch.from_numpy(obs_array).reshape(1,-1)  # Reshape the tensor to desired shape

            action = self.model.get_actions(obs_tensor)

            skills = self.skills
            num_skills = self.num_skills

            if self.stiffness_mode:
                normalized_pos = np.array(action[0][num_skills+3:num_skills+6]) # num_skills+3 and +6 since there are 3 impedance parameters representing translational stiffness
            else:
                normalized_pos = np.array(action[0][num_skills:num_skills+3])	

            unnormalized_pos = self._get_unnormalized_pos(
                normalized_pos, self.global_xyz_bounds)

            # Adjusts according to the lab measurements and simulation assumptions (such as table height, and table center is co-ordinate frame)
            table_center_robot = 0.55
            unnormalized_pos[0] = unnormalized_pos[0] + table_center_robot #+ 0.55
            #unnormalized_pos[1] = unnormalized_pos[1] + 0.55
            unnormalized_pos[2] = unnormalized_pos[2] - 0.8

            if self.stiffness_mode:
                action[0][num_skills+3:num_skills+6] = unnormalized_pos  # num_skills+3 and +6 since there are 3 impedance parameters representing translational stiffness
            else:
                action[0][num_skills:num_skills+3] = unnormalized_pos

            if self.previous_action is not None and self.are_actions_equal(action, self.previous_action):         # Check if the new action is the same as the previous action
                print("No new action.")
                print('################################')
            else:
                primitive = np.round(action[0][:num_skills])
                # Extract the primitive so you publish to the right topic

                if num_skills>1:
                    skill_index = np.where(primitive==1)
                    current_skill = skills[skill_index[0][0]]
                else:
                    current_skill = skills[0]

                primitive_params = np.round(action[0][num_skills:],3)

                if self.stiffness_mode:
                    # Unnormalizing
                    stiffness_msg = Float32MultiArray()
                    translational_stiffness_X = round(((primitive_params[0] + 1) / 2) * (self.stiffness_upper_bound - self.stiffness_lower_bound) + self.stiffness_lower_bound)
                    translational_stiffness_Y = round(((primitive_params[1] + 1) / 2) * (self.stiffness_upper_bound - self.stiffness_lower_bound) + self.stiffness_lower_bound)
                    translational_stiffness_Z = round(((primitive_params[2] + 1) / 2) * (self.stiffness_upper_bound - self.stiffness_lower_bound) + self.stiffness_lower_bound)

                    stiffness_msg.data = [translational_stiffness_X,translational_stiffness_Y,translational_stiffness_Z]
                    self.stiffness_pub.publish(stiffness_msg)

                print("Primitive Name:", current_skill)
                print('Primitive Type', primitive)
                print('Action', primitive_params)
                print('Unnormalized Position', unnormalized_pos)

                action_msg = Float64MultiArray(data=primitive_params.flatten().tolist())  # Convert NumPy array to list
                if self.stiffness_mode:
                    print('Normalized Stiffness', primitive_params[:3])
                    print('Unnormalized Stiffness',[translational_stiffness_X,translational_stiffness_Y,translational_stiffness_Z])
                    action_msg = Float64MultiArray(data=primitive_params[3:].flatten().tolist())  # Convert NumPy array to list


                start_time = time.time()

                    
                if current_skill=='reach':
#                    input("Press Enter to publish REACH primitive...")
                    self.reach_pub.publish(action_msg)
                if current_skill=='push':
                    push_temp = action_msg.data[-3:]
                    push_temp = np.clip(push_temp, -1, 1)
                    push_temp *= np.array([0.25, 0.25, 0.05])
#                    input("Press Enter to publish PUSH primitive...")
                    self.push_pub.publish(action_msg)
                if current_skill=='grasp':
#                    input("Press Enter to publish GRASP primitive...")
                    self.grasp_pub.publish(action_msg)
                # No need to open (say we don't want to drop it)
                # if current_skill=='open':
                #     self.open_pub.publish(action_msg)

                self.action_pub.publish(action_msg)

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
