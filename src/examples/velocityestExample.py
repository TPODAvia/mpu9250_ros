import os
import sys
import time
import smbus
import json
import math
import numpy as np
from scipy.linalg import norm

sys.path.append("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/src")
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick
from imusensor.filters import complimentary
from imusensor.filters import speed_estimation

sensorfusion = madgwick.Madgwick(0.5)
compl_filter = complimentary.Complimentary(0.001)
speed_est = speed_estimation.Speed_Estimation()

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# imu.caliberateGyro()
# imu.caliberateAccelerometer()
# or load your own caliberation file
imu.loadCalibDataFromFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json")

with open('/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/yaw_calib.json', 'r') as f:
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
IMU_Norm = [0, 0, 0]
NorthVector = [0,0,0]

def find_orthogonal(vector):
    # Normalize the input vector
    normalized = vector / norm(vector)
    
    # Create two potential orthogonal vectors
    u = np.array([normalized[1], -normalized[0]])
    v = np.array([-normalized[2], 0, normalized[0]])
    
    # Choose the one with the largest magnitude
    if norm(u) > norm(v):
        return u / norm(u)
    else:
        return v / norm(v)
	
def project_to_plane(input_vector, orthogonal_vector):
    # Find the projection matrix
    P = np.eye(3) - np.outer(orthogonal_vector, orthogonal_vector)
    
    # Project the input vector onto the plane
    projected_vector = P @ input_vector
    
    return projected_vector[:2] # Return only the first two components

def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

def get_quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.
	
	Input
		:param roll: The roll (rotation around x-axis) angle in radians.
		:param pitch: The pitch (rotation around y-axis) angle in radians.
		:param yaw: The yaw (rotation around z-axis) angle in radians.
	
	Output
		:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
	qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
	qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
	qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
	
	return [qx, qy, qz, qw]

def calculate_north_vector(magnetometer_data):
    # Convert the magnetometer data to radians
    x = magnetometer_data[0]
    y = magnetometer_data[1]

    # Calculate the North vector
    magnitude = math.sqrt(x**2 + y**2)
    angle = math.atan2(y, x)

    return [magnitude * math.cos(angle), magnitude * math.sin(angle), 0.0]

mag = [0,0,0]

slow_vec = [0,0]

def normalize_angles(roll, pitch, yaw):
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    # Normalize to range [-1, 1]
    norm_roll = math.sin(roll)
    norm_pitch = math.sin(pitch)
    norm_yaw = math.sin(yaw)
    
    return [norm_roll, norm_pitch, norm_yaw]

def magToHeading2(magnetic):
    heading = -1

    if magnetic[1] > 0:
        heading = 90-math.atan(magnetic[0]/magnetic[1]) * 180 / math.pi
    elif magnetic[1] < 0:
        heading = 270-math.atan(magnetic[0]/magnetic[1]) * 180 / math.pi
    elif magnetic[1] == 0:
        if magnetic[0] < 0:
            heading = 180.0
        else:
            heading = 0.0
    
    return round(heading,3)

while True:
	imu.readSensor()
	for i in range(10):
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime

		# sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
		# 							imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

		IMU_Norm = imu.AccelVals * 1/9.8
		
		if imu.AccelVals[2] < 0:
			pitch = math.atan(IMU_Norm[0] / math.sqrt(IMU_Norm[1]**2 + IMU_Norm[2]**2))
			if pitch > 0:
				pitch = 3.1 - pitch
			else:
				pitch = -3.14 - pitch
			roll = math.atan(IMU_Norm[1] / math.sqrt(IMU_Norm[0]**2 + IMU_Norm[2]**2))
			if roll > 0:
				roll = 3.1 - roll
			else:
				roll = -3.14 - roll
		else:
			pitch = math.atan(IMU_Norm[0] / math.sqrt(IMU_Norm[1]**2 + IMU_Norm[2]**2))
			roll = math.atan(IMU_Norm[1] / math.sqrt(IMU_Norm[0]**2 + IMU_Norm[2]**2))

		# Convert from radians to degrees
		pitch = math.degrees(pitch)
		roll = math.degrees(roll)

		compl_filter.updateRollAndPitch(roll, pitch, imu.GyroVals[0], -imu.GyroVals[1], dt)
            
		roll_rad = math.radians(compl_filter.roll)
		pitch_rad = math.radians(compl_filter.pitch)
		yaw_rad = math.radians(0)

		norm_angle = normalize_angles(compl_filter.roll, compl_filter.pitch, 0)
		# print(round(compl_filter.roll, 0))
		# print(round(compl_filter.pitch, 0))
		# print(round(imu.GyroVals[0], 3))
			
		# NorthVector = calculate_north_vector([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]])
		# print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))

		result_x = imu.MagVals[0]#/mag[0]
		result_y = imu.MagVals[1]#/mag[1]
		result_z = imu.MagVals[2]#/mag[2]
            
		h = math.sqrt(result_x * result_x + result_y * result_y + result_z * result_z)

		result_x /= h
		result_y /= h
		result_z /= h

		# Compute Euler angles
		# yaw_rad = math.atan2(-result_y, result_x)
            
		vec3d = np.array([result_x, -result_y, result_z])

		# Find a vector orthogonal to vec3d
		# orthogonal_vec = find_orthogonal(vec3d)

		# Project vec3d onto the plane orthogonal to orthogonal_vec
        

		projected_vec = project_to_plane(vec3d, np.array([imu.AccelVals[0]/9.8, imu.AccelVals[1]/9.8, imu.AccelVals[2]/9.8]))
            
		slow_vec[0] = 0.5*projected_vec[0] + (1 - 0.5)*slow_vec[0]
		slow_vec[1] = 0.5*projected_vec[1] + (1 - 0.5)*slow_vec[1]
            
		# print(f"Original 3D Vector: {vec3d}")
		# print(f"Orthogonal Vector: {norm_angle}")
        
		# print(f"Projected 2D Vector: {round(projected_vec[0],0)} {round(projected_vec[1],0)}")

		# print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(result_x,1), round(result_y,1), round(result_z,1)))

		# comp_vel = speed_est.estimate_velocity(newTime, newTime-dt, imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
		# 							imu.GyroVals[1], imu.GyroVals[2], get_quaternion_from_euler(compl_filter.roll, compl_filter.pitch, 0.0) )

		# ox, oy, oz = quaternion_to_euler(sensorfusion.q[1], sensorfusion.q[2], sensorfusion.q[3], sensorfusion.q[0])
		# print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format( round(vx, 6),  round(vy, 6),  round(vz, 6)))

	if print_count == 20:
		print(f"Projected 2D Vector: {round(slow_vec[0],2)} {round(slow_vec[1],2)}")
            
		# print(yaw_rad)
		# print(round(comp_vel[0], 2), round(comp_vel[1], 2), round(comp_vel[2], 2))
		# if yaw_compensate:
		# 	if (sensorfusion.yaw > -abs(data[0]) and sensorfusion.yaw < data[1]): 	# 0->90
		# 				yaw_est = lookup(sensorfusion.yaw,-abs(data[0]),data[1],0,90)

		# 	if (sensorfusion.yaw > data[1] and sensorfusion.yaw < data[2]): 		# 90->180
		# 				yaw_est = lookup(sensorfusion.yaw,data[1],data[2],90,180)

		# 	if (sensorfusion.yaw > data[2] and sensorfusion.yaw < data[3]): 		# 180->270
		# 				yaw_est = lookup(sensorfusion.yaw,data[2],data[3],180,270)

		# 	if (sensorfusion.yaw > data[3] and sensorfusion.yaw < 180):				# 270->360
		# 				yaw_est = lookup(sensorfusion.yaw,data[3],abs(data[0]),270,360)
						
		# 	print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(rotate_angle(sensorfusion.roll), 0), round(sensorfusion.pitch - 32, 0), round(yaw_est, 0)))
		# else:
		# 	print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(round(rotate_angle(sensorfusion.roll), 0), round(sensorfusion.pitch - 32 , 0), round(sensorfusion.yaw, 0)))
		
		# print ("mad roll: {0} ; mad pitch : {1} ; mad yaw : {2}".format(vx, vy, vz))
		print_count = 0

	print_count = print_count + 1
	time.sleep(0.001)