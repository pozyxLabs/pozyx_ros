#!/usr/bin/env python
"""
Configures the device list of a given Pozyx device (local and remote) for positioning.

This is intended to be highly customisable, but will also be made with parameters and a launch
file in the future. Again, help/suggestions/feedback on these things are highly appreciated.

Run this after running the UWB configuration.
Automatic calibration at this point in time is highly discouraged.
"""

import pypozyx
import rospy

# adding None will cause the local device to be configured for the anchors as well.
tag_ids = [None, 0x0001, 0x0002, 0x0003, 0x0004]

anchors = [pypozyx.DeviceCoordinates(0x0001, 1, pypozyx.Coordinates(0, 0, 1000)),
           pypozyx.DeviceCoordinates(0x0002, 1, pypozyx.Coordinates(5000, 0, 1000)),
           pypozyx.DeviceCoordinates(0x0003, 1, pypozyx.Coordinates(0, 5000, 1000)),
           pypozyx.DeviceCoordinates(0x0004, 1, pypozyx.Coordinates(5000, 5000, 1000))]


def set_anchor_configuration():
    rospy.init_node('uwb_configurator')
    rospy.loginfo("Configuring device list.")

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    for tag in tag_ids:
        for anchor in anchors:
            pozyx.addDevice(anchor, tag)
        if len(anchors) > 4:
            pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                        len(anchors), remote_id=tag)
            pozyx.saveRegisters(settings_registers, remote_id=tag)
        pozyx.saveNetwork(remote_id=tag)
        if tag is None:
            rospy.loginfo("Local device configured")
        else:
            rospy.loginfo("Device with ID 0x%0.4x configured." % tag)
    rospy.loginfo("Configuration completed! Shutting down node now...")


if __name__ == '__main__':
    set_anchor_configuration()
