#!/usr/bin/env python

import rospy
from dynamic_reconfigure.msg import Config
from std_msgs.msg import Float32MultiArray
from dynamic_reconfigure import client

def stiffness_callback(msg):
    # Extract stiffness values from the message
    translational_stiffness_X = msg.data[0]
    translational_stiffness_Y = msg.data[1]
    translational_stiffness_Z = msg.data[2]

    # Create a dynamic reconfigure client for the node you want to configure
    client_node_name = "/dynamic_reconfigure_compliance_param_node"
    dynamic_client = client.Client(client_node_name)

    # Set the new values for the parameters
    params = {
        'translational_stiffness_X': translational_stiffness_X,
        'translational_stiffness_Y': translational_stiffness_Y,
        'translational_stiffness_Z': translational_stiffness_Z
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
