########### Python library module for ST LIS3MDL Magnetometer ##########
#
# This module enables the Raspberry PI embedded computer to set up the
# LIS3MDL magnetometer.
#
# The datasheet for the ST LIS3MDL is available at
# [https://www.pololu.com/file/download/LIS3MDL.pdf?file_id=0J1089]
#
########################################################################

import struct
import smbus

class LIS3MDL:
    # LIS3MDL I2C address:
    LIS3MDL_ADDR    = 0x1C
    
    # LIS3MDL register addresses:
    LIS_WHO_AM_I    = 0x0F # Returns 0x3d (read only)

    LIS_CTRL_REG1   = 0x20 # Control register to enable device, set
                           # operating modes and rates for X and Y axes
    LIS_CTRL_REG2   = 0x21 # Set gauss scale
    LIS_CTRL_REG3   = 0x22 # Set operating/power modes
    LIS_CTRL_REG4   = 0x23 # Set operating mode and rate for Z-axis
    LIS_CTRL_REG5   = 0x24 # Set fast read, block data update modes

    LIS_STATUS_REG  = 0x27 # Read device status

    LIS_OUT_X_L     = 0x28 # X output, low byte
    LIS_OUT_X_H     = 0x29 # X output, high byte
    LIS_OUT_Y_L     = 0x2A # Y output, low byte
    LIS_OUT_Y_H     = 0x2B # Y output, high byte
    LIS_OUT_Z_L     = 0x2C # Z output, low byte
    LIS_OUT_Z_H     = 0x2D # Z output, high byte

    LIS_TEMP_OUT_L  = 0x2E # Temperature output, low byte
    LIS_TEMP_OUT_H  = 0x2F # Temperature output, high byte

    LIS_INT_CFG     = 0x30 # Interrupt generation config
    LIS_INT_SRC     = 0x31 # Interrupt sources config
    LIS_INT_THS_L   = 0x32 # Interrupt threshold, low byte
    LIS_INT_THS_H   = 0x33 # Interrupt threshold, high byte

    # Output registers:
    mag_registers = [
        LIS_OUT_X_L, # low byte of X value
        LIS_OUT_X_H, # high byte of X value
        LIS_OUT_Y_L, # low byte of Y value
        LIS_OUT_Y_H, # high byte of Y value
        LIS_OUT_Z_L, # low byte of Z value
        LIS_OUT_Z_H, # high byte of Z value
        ]
    mag_temp_registers = [
        LIS_TEMP_OUT_L, # low byte of temperature value
        LIS_TEMP_OUT_H, # high byte of temperature value
        ]
    
    def __init__(self, bus_ID=1):
        self.bus = smbus.SMBus(bus_ID)
        self.mag_enabled = False
        self.mag_temp_enabled = False

    def __del__(self):
        # This is called when the instance is about to be destroyed. It
        # is called a destructor. (Test this)
        try:
            # Power down magnetometer:
            self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0b00000011)
        except:
            pass

    # Enable and set up the mag and temperature sensors and determine
    # whether to auto increment registers during I2C read operations.
    def EnableLIS3MDL(self, magnetometer=True, temperature=True):
        # Disable magnetometer:
        self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0b00000011)
        # Disable temperature sensor:
        self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG1, 0b00000000)
        # Initialize flags
        self.mag_enabled = False
        self.mag_temp_enabled = False
        # Enable continuous conversion mode:
        self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG3, 0b00000000)
        # Initial value for CTRL_REG1
        ctrl_reg1 = 0b00000000
        if magnetometer:
            # Set the mag to ultra-high performance mode for x and y
            # and the output data rate to 10 Hz:
            ctrl_reg1 += 0b01110000
            # Set the full-scale to +/- 4 gauss:
            self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG2, 0b00000000)
            # Set the mag to ultra-high performance mode for z:
            self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG4, 0b00001100)
            self.mag_enabled = True
        if temperature:
            # Enable the temperature sensor:
            ctrl_reg1 += 0b10000000
            self.mag_temp_enabled = True
        # Write calculated value to the CTRL_REG1 register
        self.bus.write_byte_data(self.LIS3MDL_ADDR, self.LIS_CTRL_REG1, ctrl_reg1)

    def GetMagRawData(self):
        # Check if magnetometer has been enabled:
        if not self.mag_enabled:
            raise(Exception('Magnetometer has to be enabled first'))
        byte_list = [self.bus.read_byte_data(self.LIS3MDL_ADDR, x) for x in self.mag_registers]
        return struct.unpack('hhh', bytearray(byte_list))
        
    def GetMagTempRawData(self):
        # Check if temperature sensor has been enabled:
        if not self.mag_temp_enabled:
            raise(Exception('Temperature sensor has to be enabled first'))
        byte_list = [self.bus.read_byte_data(self.LIS3MDL_ADDR, x) for x in self.mag_temp_registers]
        return struct.unpack('h', bytearray(byte_list))[0]
