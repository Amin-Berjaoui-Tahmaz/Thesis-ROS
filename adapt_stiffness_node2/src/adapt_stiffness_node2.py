#!/usr/bin/env python

import rospy
import numpy as np
from dynamic_reconfigure.msg import Config
from std_msgs.msg import Float32MultiArray
from dynamic_reconfigure import client
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

def stiffness_callback(msg):
    
    # Extract stiffness values from the message
    translational_stiffness_X = msg.data[0]
    translational_stiffness_Y = msg.data[1]
    translational_stiffness_Z = msg.data[2]

    print('translational_stiffness_X',translational_stiffness_X)
    print('translational_stiffness_Y',translational_stiffness_Y)
    print('translational_stiffness_Z',translational_stiffness_Z)
    print('############')

    # Create a dynamic reconfigure client for the node you want to configure
    client_node_name = "/dynamic_reconfigure_compliance_param_node"
    dynamic_client = client.Client(client_node_name)

    ###################
    ### AFORCE HERE ###
    ###################

    # epsilon is arbitrary, say 8 cm then it stops changing stiffness
    epsilon = 0.08
    gamma = 9e-3 #9e-3 #10e-2#2e-3 #9e-4 #9e-4#9e-5 ####9e-3 in first ###15e-3 in second
    beta = 0.2 #0.2 # 0.14 #5000*gamma#0.2 #1.4#0.14 #0.14 in first ##0.14 in second

    AFORCE_mode = False
    position_error = -10 # arbitrary initialization
    if AFORCE_mode:
        while position_error<epsilon:

            # CHATGPT, DO THIS:
            # CONTINUOUSLY read from cartesian_pose for current_position
            # CONTINUOUSLY read from reach/grasp/push for desired_position
            # CONTINUOUSLY read from joint info to get effort (as energy consumed)
            # Make sure that this keeps updating

            # CHATGPT, READ CURRENT POSITION FROM /cartesian_pose TOPIC
            # (Assuming /cartesian_pose publishes a Float32MultiArray message with x, y, and z coordinates)
            current_position_msg = rospy.wait_for_message("/cartesian_pose", PoseStamped)
            current_position_x = current_position_msg.pose.position.x
            current_position_y = current_position_msg.pose.position.y
            current_position_z = current_position_msg.pose.position.z

            # CHATGPT, READ DESIRED POSITION FROM PARAMETERS OR ANOTHER TOPIC
            # (Assuming desired_position is available from a parameter or another topic)
            desired_position = np.array([0, 0, 0])  # Replace with actual values

            # CHATGPT, READ ENERGY CONSUMED FROM JOINT INFO
            # (Assuming joint_info publishes a Float32MultiArray message with energy consumed)
            joint_info_msg = rospy.wait_for_message("/franka_state_controller/joint_states", JointState)
            energy_consumed = abs(np.sum(joint_info_msg.effort))

            print('energy_consumed',energy_consumed)

            print('desired_position',desired_position)

            position_error_x = current_position_x - desired_position[0]
            position_error_y = current_position_y - desired_position[1]
            position_error_z = current_position_z - desired_position[2]

            position_error = np.linalg.norm([position_error_x,position_error_y,position_error_z])

            print('position_error_x',position_error_x)
            print('position_error_y',position_error_y)
            print('position_error_z',position_error_z)
            print('position_error',position_error)

            k_dot_X = beta * np.abs(position_error_x) - gamma * energy_consumed # energy_consumed from joint_effort
            k_dot_Y = beta * np.abs(position_error_y) - gamma * energy_consumed # energy_consumed from joint_effort
            k_dot_Z = beta * np.abs(position_error_z) - gamma * energy_consumed # energy_consumed from joint_effort

            translational_stiffness_X += k_dot_X
            translational_stiffness_Y += k_dot_Y
            translational_stiffness_Z += k_dot_Z

            # Set the new values for the parameters
            params = {
                'translational_stiffness_X': np.round(translational_stiffness_X,2),
                'translational_stiffness_Y': np.round(translational_stiffness_Y,2),
                'translational_stiffness_Z': np.round(translational_stiffness_Z,2)
            }

            print('translational_stiffness_X',translational_stiffness_X)
            print('translational_stiffness_Y',translational_stiffness_Y)
            print('translational_stiffness_Z',translational_stiffness_Z)
            print('######################################################')

            # Update the parameters using the dynamic reconfigure client
            dynamic_client.update_configuration(params)

    else:
        # Set the new values for the parameters
        params = {
            'translational_stiffness_X': np.round(translational_stiffness_X,3),
            'translational_stiffness_Y': np.round(translational_stiffness_Y,3),
            'translational_stiffness_Z': np.round(translational_stiffness_Z,3)
        }

        # Update the parameters using the dynamic reconfigure client
        dynamic_client.update_configuration(params)


if __name__ == '__main__':
    rospy.init_node('stiffness_listener')
    
    # Subscribe to the /stiffness_action topic
    rospy.Subscriber("/stiffness_action", Float32MultiArray, stiffness_callback)

    # Set the rate to 20Hz
    rate = rospy.Rate(20)

    rospy.spin()
