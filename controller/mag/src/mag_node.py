#!/usr/bin/env python

import rospy

from drivers.lis3mdl import LIS3MDL

from sensor_msgs.msg import MagneticField

mag = LIS3MDL()
mag.EnableLIS3MDL()

rospy.init_node('mag_node', anonymous = True)
mag_publisher = rospy.Publisher('bfield', MagneticField, queue_size = 2)

def publish_mag_state_msg(publisher):
    mag_state_msg = MagneticField()
    mag_state_msg.header.stamp = rospy.get_rostime()
    mag_state_msg.magnetic_field.x = mag.GetMagRawData()[0]
    mag_state_msg.magnetic_field.y = mag.GetMagRawData()[1]
    mag_state_msg.magnetic_field.z = mag.GetMagRawData()[2]
    publisher.publish(mag_state_msg)

if __name__ == '__main__':
    try:
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            publish_mag_state_msg(mag_publisher)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
