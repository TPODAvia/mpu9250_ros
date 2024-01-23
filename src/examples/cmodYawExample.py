import os
import sys
import time
import smbus
import json

sys.path.append("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/src/")
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
imu.loadCalibDataFromFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json")

currTime = time.time()
print_count = 0
angle = 0
yaw_data = [0,0,0,0]
yaw_est = 0.0
buffer_time = time.time() - 20
print ("Yaw dirrection estiomation is starting. Put the sensor in the horisontal and still surface. Put the first measurement in to the measutrement +-180 degrees and rotate 4 times with 90 degerees shift.")
while True:
	imu.readSensor()
	for i in range(10):
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime

		sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
									imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

	if print_count == 20:
		print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(sensorfusion.roll, 0), round(sensorfusion.pitch, 0), round(sensorfusion.yaw, 0)))
		print_count = 0

		if time.time() - buffer_time > 20:

			if angle > 0:
				yaw_data[angle-1] = sensorfusion.yaw

			if angle > 3:
				# data = {yaw_data}
				with open('/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/yaw_calib.json', 'w') as f:
					json.dump(yaw_data, f)
					print("json saved")
				angle = 0

			input("Put the IMU in {0} position. Press enter to continue..".format((angle)*90))
			angle+=1
			buffer_time = time.time()


	print_count = print_count + 1
	time.sleep(0.001)