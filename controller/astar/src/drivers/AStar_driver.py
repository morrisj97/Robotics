#!/usr/bin/env python
#
# ============ Python driver module for Pololu 32U4 Controller ===============
#
# This module enables the Raspberry PI embedded computer to send and receive
# data from the Pololu 32U4 controller. It is based on the AStar code found
# in the pololu-rpi-slave-arduino-library on GitHub. The register addresses
# are configured by the "RomiRPiSlaveMotorControl" Arduino code loaded to the
# 32U4.
#
# battery_voltage;        // 0x00-0x01 (00-01)   Battery voltage in mV (16-bit - 2 bytes)
# encoder_left;           // 0x02-0x03 (02-03)   Motor pulse width commands 
# encoder_right;          // 0x04-0x05 (04-05)   (16-bit - 2 bytes each)
# motor_speed_left;       // 0x06-0x07 (06-07)   Measured motor speeds in counts/sec 
# motor_speed_right;      // 0x08-0x09 (08-09)   (16-bit - 2 bytes each)
# velocity_trans;         // 0x0A-0x0D (10-13)   Translational and angular velocities
# velocity_ang;           // 0x0E-0x11 (14-17)   (32-bit - 4 bytes each)
# position_x;             // 0x12-0x15 (18-21)   Pose - X and Y position and angle
# position_y;             // 0x16-0x19 (22-25)   (32-bit - 4 bytes each)
# theta;                  // 0x1A-0x1D(26-29)
# desired_velocity_trans; // 0x1E-0x21 (30-33)   Target translational and angular velocities
# desired_velocity_ang;   // 0x22-0x25 (34-37)   (32-bit - 4 bytes each)
# encoder_reset;          // 0x26      (38)      Encoder reset flag (8-bit - 1 byte)
#
# The user's guide for the Romi 32U4 Control Board is available at
# [https://www.pololu.com/docs/pdf/0J69/romi_32u4_control_board.pdf]
#
# Notes:
# read_unpack, write_pack types quick reference
# ? - bool           - 1
# B - unsigned char  - 1
# h - short          - 2
# H - unsigned short - 2
# f - float          - 4
#
# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

import smbus
import struct
import time
import math

I2C_BUS_ADDRESS = 0x14

# Constants:
COUNTS_PER_REV = 1440.0

class AStar:
    def __init__(self):
        self.bus = smbus.SMBus(1)

    def close(self):
        self.bus.close()
        
    def read_raw(self, i2c_address, size):
        byte_list = [self.bus.read_byte(i2c_address) for _ in range(size)]
        return byte_list

    def read_unpack(self, i2c_address, reg_address, size, data_format):
        self.bus.write_byte(i2c_address, reg_address)
        time.sleep(0.0002)
        byte_list = [self.bus.read_byte(i2c_address) for _ in range(size)]
        return struct.unpack(data_format, bytearray(byte_list))

    def write_pack(self, i2c_address, reg_address, data_format, *data):
        data_array = []
        byte_list = struct.pack(data_format, *data)
        for byte in byte_list:
                data_array.append(ord(byte))
        self.bus.write_i2c_block_data(i2c_address, reg_address, data_array)
        time.sleep(0.0001)

    def read_battery_millivolts(self):
        return self.read_unpack(I2C_BUS_ADDRESS, 0x00, 2, "h")[0]
 
    def read_wheel_positions(self): # radians
        encoder_values = self.read_unpack(I2C_BUS_ADDRESS, 0x02, 4, 'hh')
        if( encoder_values is None ):
            return (None,None)
        left, right = encoder_values
        return (2 * math.pi * left / COUNTS_PER_REV, 2 * math.pi * right / COUNTS_PER_REV)
        
    def read_wheel_speeds(self): # radians/sec
        left, right = self.read_unpack(I2C_BUS_ADDRESS, 0x06, 4, 'hh')
        return (2 * math.pi * left / COUNTS_PER_REV, 2 * math.pi * right / COUNTS_PER_REV)
        
    def read_pose_twist(self):
        return self.read_unpack(I2C_BUS_ADDRESS, 0x0A, 8, 'ff')

    def read_pose_coordinates(self):
        x, y, theta = self.read_unpack(I2C_BUS_ADDRESS, 0x12, 12, "fff")
        return (x,y,theta)
        
##    def _new_twist(self):
##        self.write_pack(I2C_BUS_ADDRESS, 0x26,'?', True)
        
    def twist(self, trans, ang):
        self.write_pack(I2C_BUS_ADDRESS, 0x1E, 'ff', trans, ang)
##        self._new_twist()

    def read_twist(self):
        return self.read_unpack(I2C_BUS_ADDRESS, 0x1E, 8, 'ff')

##    def print_debug_info(self):
##        print "== RomiPi Debug Info ============================="
##        print "Battery:          ", self.read_battery_millivolts(), " mV"
##        print "Encoders (l,r):  ", self.read_encoders()
##        print "Estimated motor speeds (l,r): ", self.read_motor_speeds()
##        print "Estimated Twist (translational m/s, rotation rad/s): %0.2f, %0.2f" % self.read_pose_twist()
##        print "Estimated Pose (x m,y m, theta rad): %0.1f, %0.1f %0.1f" % self.read_pose_coordinates()
##        print "Commanded Twist (translational m/s, rotation rad/s): %0.1f, %0.1f" % self.read_twist()

    def square(self):
        for i in range(4):
            self.twist(0.2,0.0)
            time.sleep(1.0)
            self.twist(0.0, 3.14/2)
            time.sleep(1.0)
            
    def line(self):
        self.twist(0.2,0.0)
        time.sleep(1.0)
        self.twist(0.0, 3.14)
        time.sleep(1.0)
        self.twist(0.2,0.0)
        time.sleep(1.0)
       
    def circle(self):
        self.twist(2*3.14*0.2/8,2*3.14/8)
        time.sleep(8.0)

if __name__ == '__main__':
        romi = AStar()
##        romi.print_debug_info()
        # romi.reset_encoders()
        while 1==1:
##                print "Encoders (l,r):  ", romi.read_encoders()
                # try:
                romi.twist(0,0)
#                romi.square()
                # except:
                        # pass
                        # romi.twist(0,0)
