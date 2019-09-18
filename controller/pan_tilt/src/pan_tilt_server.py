#!/usr/bin/python

import rospy
import RPi.GPIO as GPIO
from drivers.PCA9685 import PCA9685
from pan_tilt.msg import PanTilt

class PanTiltServer(object):
    def __init__(self):
        rospy.loginfo("Starting Pan and Tilt Server...")
        self.Pan_Tilt_PWM = PCA9685()
        self.Pan_Tilt_PWM.setPWMFreq(50)
        self.Pan_Tilt_PWM.move_to_pitch_yaw(yaw=90, pitch=45)
        Pan_Tilt_Subscriber = rospy.Subscriber('/pan_tilt', PanTilt, self.Pan_Tilt_Callback)
        rospy.loginfo("Pan and Tilt Server...READY")
    def __del__(self):
        self.Pan_Tilt_PWM.exit_PCA9685()
        GPIO.cleanup()
    def Pan_Tilt_Callback(self, msg):
        self.Pan_Tilt_PWM.move_to_pitch_yaw(yaw=msg.pan, pitch=msg.tilt)

if __name__ == "__main__":
    rospy.init_node('Pan_Tilt_Server')
    Pan_Tilt_Server = PanTiltServer()
    rospy.spin()
