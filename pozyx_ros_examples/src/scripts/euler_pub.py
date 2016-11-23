#!/usr/bin/env python
"""
ROS node that publishes an IMU data example through the Euler Angles

Expanding to more functionality can easily be done, and there's also
getAllSensorData.
"""

import pypozyx
import rospy
from pozyx_ros_examples.msg import EulerAngles

remote_id = None


def pozyx_euler_pub():
    pub = rospy.Publisher('pozyx_euler_angles', EulerAngles, queue_size=100)
    rospy.init_node('pozyx_euler_pub')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        euler_angles = pypozyx.EulerAngles()
        pozyx.getEulerAngles_deg(euler_angles, remote_id=remote_id)
        rospy.loginfo(euler_angles)
        pub.publish(euler_angles.heading,
                    euler_angles.roll, euler_angles.pitch)


if __name__ == '__main__':
    try:
        pozyx_euler_pub()
    except rospy.ROSInterruptException:
        pass
