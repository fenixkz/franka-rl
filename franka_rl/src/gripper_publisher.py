#!/usr/bin/env python
"""
A simple script that translates desired gripper width to command for
JointGroupPositionController.
"""
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64



if __name__ == '__main__':
    rospy.init_node('gripper_publisher')
    # spin() simply keeps python from exiting until this node is stopped
    pub = rospy.Publisher('/franka/gripper_position_controller/command',
                          Float64MultiArray, queue_size=10)

    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension('', 2, 1)]
    msg.data = [0.2, 0.5]
    print("Moving...")
    pub.publish(msg)
