#!/usr/bin/env python

# rospy is the ROS client library for Python. 
import rospy
import math

# Import the AStar class from the romipi_driver.py script:
from drivers.AStar_driver import AStar

from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

romi = AStar()
# Before starting an ROS node, the first function called initializes the node. The
# first argument is the name of the node, and anonymous = True means the node can
# run on multiple instances.
rospy.init_node('romipi_astar_node', anonymous = True)

# rospy.Publisher creates a handle to publish messages to a topic.
# Syntax: rospy.Publisher(topic_name, msg_class, queue_size)
battery_publisher = rospy.Publisher('battery', BatteryState, queue_size = 2)
wheel_publisher = rospy.Publisher('wheels', JointState, queue_size = 2)
odometry_publisher = rospy.Publisher('odom', Odometry, queue_size = 2)

def publish_battery_state_msg(publisher):
    # The Romi is powered by six 1.5V 2400 mAh AA NiMH batteries. Their combined
    # operating voltage is about 7.2V.
    # Define the message class for battery_state_msg:
    battery_state_msg = BatteryState()
    # Time tag:
    battery_state_msg.header.stamp = rospy.get_rostime()
    # Capacity in amp-hours:
    battery_state_msg.design_capacity = 2.4
    # Read the battery voltage over the I2C interface:
    battery_state_msg.voltage = romi.read_battery_millivolts() / 1000.0
    if( romi.read_battery_millivolts() >= 0.00 ):
        battery_state_msg.present = True
    else:
        battery_state_msg.present = False
    # Publish the message
    publisher.publish(battery_state_msg)

def publish_wheel_state_msg(publisher):
    # Define the message class for wheel_state_msg:
    wheel_state_msg = JointState()
    # Time tag:
    wheel_state_msg.header.stamp = rospy.get_rostime()
    # All the arrays in this message should have the same size.
    # Name the two elements of the arrays:
    wheel_state_msg.name = ["left_wheel", "right_wheel"]
    # Read the wheel position in radians:
    wheel_state_msg.position = romi.read_wheel_positions()
    # Read the wheel rotational rates in radians/sec:
    wheel_state_msg.velocity = romi.read_wheel_speeds()
    # Set the effort to zero since torque is not measured:
    wheel_state_msg.effort = []
    # Publish the message
    publisher.publish(wheel_state_msg)

def publish_odometry_state_msg(publisher):
    # Define the message class for odometry_state_msg:
    odometry_state_msg = Odometry()

    odometry_state_msg.header.frame_id = "odom"
    odometry_state_msg.child_frame_id = "base_link"
    
    # Time tag:
    odometry_state_msg.header.stamp = rospy.get_rostime()
    # The pose is specified in the coordinate frame given by:
    odometry_state_msg.pose.pose.position.x = romi.read_pose_coordinates()[0]
    odometry_state_msg.pose.pose.position.y = romi.read_pose_coordinates()[1]
    odometry_state_msg.pose.pose.position.z = 0.0
    odometry_state_msg.pose.pose.orientation.x = 0.0
    odometry_state_msg.pose.pose.orientation.y = 0.0
    odometry_state_msg.pose.pose.orientation.z = math.sin(romi.read_pose_coordinates()[2] / 2)
    odometry_state_msg.pose.pose.orientation.w = math.cos(romi.read_pose_coordinates()[2] / 2)
    odometry_state_msg.pose.covariance[0] = 1e-10
    odometry_state_msg.pose.covariance[7] = 1e-10
    odometry_state_msg.pose.covariance[14] = 1e-10
    odometry_state_msg.pose.covariance[21] = 1e-10
    odometry_state_msg.pose.covariance[28] = 1e-10
    odometry_state_msg.pose.covariance[35] = 1e-10

    # The twist is specified in the coordinate frame given by:
    odometry_state_msg.twist.twist.linear.x = romi.read_pose_twist()[0]
    odometry_state_msg.twist.twist.angular.z = romi.read_pose_twist()[1]
    odometry_state_msg.twist.covariance[0] = 1e-10
    odometry_state_msg.twist.covariance[7] = 1e-10
    odometry_state_msg.twist.covariance[14] = 1e-10
    odometry_state_msg.twist.covariance[21] = 1e-10
    odometry_state_msg.twist.covariance[28] = 1e-10
    odometry_state_msg.twist.covariance[35] = 1e-10

    # Publish the message
    publisher.publish(odometry_state_msg)

def cmd_vel_callback(data):
    romi.twist(data.linear.x, data.angular.z)

if __name__ == '__main__':
    try:
        rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            publish_battery_state_msg(battery_publisher)
            publish_wheel_state_msg(wheel_publisher)
            publish_odometry_state_msg(odometry_publisher)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
