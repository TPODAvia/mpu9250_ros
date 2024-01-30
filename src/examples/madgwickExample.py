import os
import sys
import time
import smbus
import json

sys.path.append("/home/rover/gr_platform/ros/src/sensing/mpu9250_ros/src/")
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick

sensorfusion = madgwick.Madgwick(0.5)

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# imu.caliberateGyro()
# imu.caliberateAccelerometer()
# or load your own caliberation file
imu.loadCalibDataFromFile("/home/rover/gr_platform/ros/src/sensing/mpu9250_ros/config/calib.json")

with open('/home/rover/gr_platform/ros/src/sensing/mpu9250_ros/config/yaw_calib.json', 'r') as f:
    data = json.load(f)

def lookup(x, old_min, old_max, new_min, new_max):
    return ((new_max - new_min) * (x - old_min) / (old_max - old_min)) + new_min

# Shint in counteclockwise by 44 degrees
def rotate_angle(angle):
    shifted_angle = (angle + 134) % 360
    if shifted_angle > 180:
        shifted_angle -= 360
    return shifted_angle

currTime = time.time()
print_count = 0
yaw_compensate = False

while True:
	imu.readSensor()
	for i in range(10):
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime

		sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
									imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

	if print_count == 20:

		if yaw_compensate:
			if (sensorfusion.yaw > -abs(data[0]) and sensorfusion.yaw < data[1]): 	# 0->90
						yaw_est = lookup(sensorfusion.yaw,-abs(data[0]),data[1],0,90)

			if (sensorfusion.yaw > data[1] and sensorfusion.yaw < data[2]): 		# 90->180
						yaw_est = lookup(sensorfusion.yaw,data[1],data[2],90,180)

			if (sensorfusion.yaw > data[2] and sensorfusion.yaw < data[3]): 		# 180->270
						yaw_est = lookup(sensorfusion.yaw,data[2],data[3],180,270)

			if (sensorfusion.yaw > data[3] and sensorfusion.yaw < 180):				# 270->360
						yaw_est = lookup(sensorfusion.yaw,data[3],abs(data[0]),270,360)
						
			print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(rotate_angle(sensorfusion.roll), 0), round(sensorfusion.pitch - 32, 0), round(yaw_est, 0)))
		else:
			print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(rotate_angle(sensorfusion.roll), 0), round(sensorfusion.pitch - 32 , 0), round(sensorfusion.yaw, 0)))

		print_count = 0

	print_count = print_count + 1
	time.sleep(0.001)