#!/usr/bin/env python
"""ROS node that performs 3D positioning on Pozyx"""

import pypozyx
import rospy
from geometry_msgs.msg import Point32

remote_id = None


def pozyx_positioning_pub():
    pub = rospy.Publisher('pozyx_positioning', Point32, queue_size=100)
    rospy.init_node('positioning_pub')
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        pozyx.doPositioning(coords, remote_id=remote_id)
        rospy.loginfo(coords)
        pub.publish(Point32(coords.x, coords.y, coords.z))


if __name__ == '__main__':
    try:
        pozyx_positioning_pub()
    except rospy.ROSInterruptException:
        pass
