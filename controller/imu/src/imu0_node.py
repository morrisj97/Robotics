#!/usr/bin/env python

import rospy
import math

from drivers.lsm6ds33 import LSM6DS33

from sensor_msgs.msg import Imu

imu = LSM6DS33(1, 0x6A)
accel_bias_x = 0.0034

gyro_scalefactor = (math.pi / 180) * 4.374 / 1000 # rad/sec/LSB
accel_scalefactor = 9.80665 * 0.122 / 1000 # m/s2/LSB 

linear_acceleration_stdev = 9.80665 * 4.0e-3
angular_velocity_stdev = (math.pi / 180) * 0.06

linear_acceleration_covar = linear_acceleration_stdev * linear_acceleration_stdev
angular_velocity_covar = angular_velocity_stdev * angular_velocity_stdev

imu.EnableLSM6DS33()

rospy.init_node('imu_node', anonymous = True)
imu_publisher = rospy.Publisher('imu0_data', Imu, queue_size = 2)

def publish_imu_state_msg(publisher):
    imu_state_msg = Imu()
    imu_state_msg.header.frame_id = "base_link"
    imu_state_msg.header.stamp = rospy.get_rostime()
    imu_state_msg.angular_velocity.x = imu.GetGyroRawData()[0] * gyro_scalefactor
    imu_state_msg.angular_velocity.y = imu.GetGyroRawData()[1] * gyro_scalefactor
    imu_state_msg.angular_velocity.z = imu.GetGyroRawData()[2] * gyro_scalefactor

    imu_state_msg.angular_velocity_covariance[0] = angular_velocity_covar
    imu_state_msg.angular_velocity_covariance[4] = angular_velocity_covar
    imu_state_msg.angular_velocity_covariance[8] = angular_velocity_covar

    imu_state_msg.linear_acceleration.x = imu.GetAccelRawData()[0] * accel_scalefactor - accel_bias_x
    imu_state_msg.linear_acceleration.y = imu.GetAccelRawData()[1] * accel_scalefactor
    imu_state_msg.linear_acceleration.z = imu.GetAccelRawData()[2] * accel_scalefactor

    imu_state_msg.linear_acceleration_covariance[0] = linear_acceleration_covar
    imu_state_msg.linear_acceleration_covariance[4] = linear_acceleration_covar
    imu_state_msg.linear_acceleration_covariance[8] = linear_acceleration_covar

    publisher.publish(imu_state_msg)

if __name__ == '__main__':
    try:
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            publish_imu_state_msg(imu_publisher)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
