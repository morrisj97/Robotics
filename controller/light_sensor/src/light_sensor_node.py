#!/usr/bin/python

import rospy
from drivers.TSL2581 import TSL2581
from light_sensor.msg import Lux

light = TSL2581(0X39, debug=False)
light.Init_TSL2581()

rospy.init_node('light_sensor_node', anonymous = True)
light_sensor_publisher = rospy.Publisher('lux', Lux, queue_size = 2)

def publish_light_measurement_msg(publisher):
    lux_value_msg = Lux()
    lux_value_msg.header.stamp = rospy.get_rostime()
    lux_value_msg.lux = light.calculate_Lux()
    publisher.publish(lux_value_msg)

if __name__ == '__main__':
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            publish_light_measurement_msg(light_sensor_publisher)
            rate.sleep()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
