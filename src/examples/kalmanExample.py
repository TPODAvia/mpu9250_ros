import os
import sys
import time
import smbus
import numpy as np

sys.path.append("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/src/")
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
# imu.caliberateAccelerometer()
# print ("Acceleration calib successful")
# imu.caliberateMag()
# print ("Mag calib successful")
# or load your calibration file
# imu.loadCalibDataFromFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/parameters.json")

sensorfusion = kalman.Kalman()

imu.readSensor()
imu.computeOrientation()
sensorfusion.roll = imu.roll
sensorfusion.pitch = imu.pitch
sensorfusion.yaw = imu.yaw

count = 0
currTime = time.time()
while True:
	imu.readSensor()
	imu.computeOrientation()
	newTime = time.time()
	dt = newTime - currTime
	currTime = newTime

	sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],\
												imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
	
	# print(imu.MagVals[2])

	print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(round(sensorfusion.roll, 0), round(sensorfusion.pitch, 0), round(sensorfusion.yaw, 0)))

	time.sleep(0.05)