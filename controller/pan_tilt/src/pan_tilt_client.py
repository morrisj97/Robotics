#!/usr/bin/python
import sys
import rospy
import math
import time
from pan_tilt.msg import PanTilt



class PanTiltClient(object):

    def __init__(self):
        self._pan_tilt_publisher = rospy.Publisher('/pan_tilt', PanTilt, queue_size=1)

    def pan_tilt_move(self, pan, tilt):
        pan_tilt_msg = PanTilt()
        pan_tilt_msg.pan = pan
        pan_tilt_msg.tilt = tilt
        self._pan_tilt_publisher.publish(pan_tilt_msg)

def yaw_pitch_circle_test(repetitions=1):
    pan_tilt_client = PanTiltClient()
    max_range = 100
    min_range = 30
    period = 0.02
    increments = 1
    for num in range(repetitions):
        for angle in range(0,359,increments):
            pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            pan_tilt_client.pan_tilt_move(pan=yaw_angle, tilt=pitch_angle)
            time.sleep(period)

        for angle in range(0,359,-increments):
            pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
            pan_tilt_client.pan_tilt_move(pan=yaw_angle, tilt=pitch_angle)
            time.sleep(period)

def raw_input_pan_tilt_test():
    pan_tilt_client = PanTiltClient()

    while not rospy.is_shutdown():
        print(">>>>>>New Input>>>>>>>>>>\n")
        pan = float(raw_input("Input Pan=>"))
        tilt = float(raw_input("Input Tilt=>"))
        pan_tilt_client.pan_tilt_move(pan=pan, tilt=tilt)
        print(">>>>>>@@@@@@@@@>>>>>>>>>>\n")


if __name__ == "__main__":
    rospy.init_node('pan_tilt_client')
    yaw_pitch_circle_test(repetitions=10)
    #raw_input_pan_tilt_test()
