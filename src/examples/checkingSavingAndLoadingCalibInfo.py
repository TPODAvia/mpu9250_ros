import os
import sys
import time
import smbus
import numpy as np

sys.path.append("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/src/")
from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.caliberateAccelerometer()
print ("Acceleration calib successful")
print ("Mag calibration starting")
time.sleep(2)
imu.caliberateMagPrecise()
print ("Mag calib successful")

accelscale = imu.Accels
accelBias = imu.AccelBias
gyroBias = imu.GyroBias
mags = imu.Mags 
magBias = imu.MagBias

imu.saveCalibDataToFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json")
print ("calib data saved")

imu.loadCalibDataFromFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json")
if np.array_equal(accelscale, imu.Accels) & np.array_equal(accelBias, imu.AccelBias) & \
	np.array_equal(mags, imu.Mags) & np.array_equal(magBias, imu.MagBias) & \
	np.array_equal(gyroBias, imu.GyroBias):
	print ("calib loaded properly")
