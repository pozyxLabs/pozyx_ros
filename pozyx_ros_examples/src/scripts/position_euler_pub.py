#!/usr/bin/env python
"""
ROS node that performs positioning and Euler measurement on Pozyx

This shows how to separate the position and IMU data in two separate channels.
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point32
from pozyx_ros_examples.msg import EulerAngles

remote_id = None


def pozyx_position_euler_pub():
    position_pub = rospy.Publisher(
        'pozyx_positioning', Point32, queue_size=100)
    euler_pub = rospy.Publisger(
        'pozyx_euler_angles', EulerAngles, queue_size=100)
    rospy.init_node('position_euler_pub')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        pozyx.doPositioning(coords, remote_id=remote_id)
        pub.publish(coords.x, coords.y, coords.z)
        euler_angles = pypozyx.EulerAngles()
        pozyx.getEulerAngles_deg(euler_angles)
        rospy.loginfo("POS: %s, ANGLES: %s" % (str(coords), str(euler_angles)))
        pub.publish(euler_angles.heading,
                    euler_angles.roll, euler_angles.pitch)


if __name__ == '__main__':
    try:
        pozyx_positioning_euler_pub()
    except rospy.ROSInterruptException:
        pass
