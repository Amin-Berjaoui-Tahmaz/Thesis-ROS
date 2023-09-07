#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from franka_core_msgs.msg import JointCommand

class PoseInterpolatorNode:
    def __init__(self):
        rospy.init_node('pose_interpolator_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz rate for interpolation
        self.listener = tf.TransformListener()

        self.desired_pose = None
        self.current_pose = None

        # Publishers
        self.equilibrium_pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber('/equilibrium_pose_uninterpolated', PoseStamped, self.desired_pose_callback)

    def desired_pose_callback(self, pose_msg):
        self.desired_pose = pose_msg

    def interpolate_poses(self):
        while not rospy.is_shutdown():
            if self.desired_pose:
                try:
                    (trans, rot) = self.listener.lookupTransform('/panda_link0', '/panda_link8', rospy.Time(0))

                    current_pose = geometry_msgs.msg.Pose()
                    current_pose.position.x = trans[0]
                    current_pose.position.y = trans[1]
                    current_pose.position.z = trans[2]
                    current_pose.orientation.x = rot[0]
                    current_pose.orientation.y = rot[1]
                    current_pose.orientation.z = rot[2]
                    current_pose.orientation.w = rot[3]

                    if self.current_pose is not None:
                        # Perform linear interpolation between the current and desired pose
                        alpha = 0.1  # You can adjust this value for the interpolation speed
                        interpolated_pose = geometry_msgs.msg.Pose()
                        interpolated_pose.position.x = (1 - alpha) * current_pose.position.x + alpha * self.desired_pose.position.x
                        interpolated_pose.position.y = (1 - alpha) * current_pose.position.y + alpha * self.desired_pose.position.y
                        interpolated_pose.position.z = (1 - alpha) * current_pose.position.z + alpha * self.desired_pose.position.z
                        interpolated_pose.orientation.x = (1 - alpha) * current_pose.orientation.x + alpha * self.desired_pose.orientation.x
                        interpolated_pose.orientation.y = (1 - alpha) * current_pose.orientation.y + alpha * self.desired_pose.orientation.y
                        interpolated_pose.orientation.z = (1 - alpha) * current_pose.orientation.z + alpha * self.desired_pose.orientation.z
                        interpolated_pose.orientation.w = (1 - alpha) * current_pose.orientation.w + alpha * self.desired_pose.orientation.w

                        print('interpolated_pose', interpolated_pose)
                        print('##################')

                        # Publish the interpolated pose
                        self.equilibrium_pose_pub.publish(interpolated_pose)
                        self.current_pose = interpolated_pose
                    else:
                        self.current_pose = current_pose

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            self.rate.sleep()

if __name__ == '__main__':
    try:
        pose_interpolator_node = PoseInterpolatorNode()
        pose_interpolator_node.interpolate_poses()
    except rospy.ROSInterruptException:
        pass
