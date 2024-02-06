import os
import sys
import time
import smbus
import json
import math
import numpy as np

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

def calculate_pitch_and_roll(accel_vals):
    IMU_Norm = accel_vals * (1/9.8)
    
    if accel_vals[2] <  0:
        pitch = math.atan(IMU_Norm[0] / math.sqrt(IMU_Norm[1]**2 + IMU_Norm[2]**2))
        if pitch >  0:
            pitch =  3.1 - pitch
        else:
            pitch = -3.14 - pitch
        roll = math.atan(IMU_Norm[1] / math.sqrt(IMU_Norm[0]**2 + IMU_Norm[2]**2))
        if roll >  0:
            roll =  3.1 - roll
        else:
            roll = -3.14 - roll
    else:
        pitch = math.atan(IMU_Norm[0] / math.sqrt(IMU_Norm[1]**2 + IMU_Norm[2]**2))
        roll = math.atan(IMU_Norm[1] / math.sqrt(IMU_Norm[0]**2 + IMU_Norm[2]**2))

    # Convert from radians to degrees
    pitch = math.degrees(pitch)
    roll = math.degrees(roll)
    
    return pitch, roll

def quaternion_to_euler(q):

	x = q[1]
	y = q[2]
	z = q[3]
	w = q[0]

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

	return [X, Y, Z]

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [w, x, y, z]

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

def calculate_ned_frame(magnetic_field_vector, gravity_vector):
    # Normalize the gravity vector to get the downward direction
    down_vector = gravity_vector / np.linalg.norm(gravity_vector)

    # Normalize the magnetic field vector to get the magnetic north direction
    north_vector = magnetic_field_vector / np.linalg.norm(magnetic_field_vector)

    # Calculate the east vector as the cross product of down and north
    east_vector = np.cross(down_vector, north_vector)

    # Recompute the north vector as the cross product of east and down
    # to ensure it's perpendicular to both
    north_vector = np.cross(east_vector, down_vector)

    return [down_vector, north_vector, east_vector]


def calculate_quaternion_from_coordinates(initial_vector, ortogonal_vector1, ortogonal_vector2):
    
    desired_vector = [ortogonal_vector1[0]+ortogonal_vector2[1],ortogonal_vector1[1]-ortogonal_vector2[0],0]
    
    # Normalize the initial and final vectors
    initial_norm = initial_vector / np.linalg.norm(initial_vector)
    final_norm = desired_vector / np.linalg.norm(desired_vector)
    
    # print(final_norm)
    
    # Calculate the halfway vector
    halfway = final_norm + initial_norm
    
    # Normalize the halfway vector
    halfway_norm = halfway / np.linalg.norm(halfway)
    
    # Compute the quaternion components
    w = np.dot(initial_norm, halfway_norm)
    x = initial_norm[1]*halfway_norm[2] - initial_norm[2]*halfway_norm[1]
    y = initial_norm[2]*halfway_norm[0] - initial_norm[0]*halfway_norm[2]
    z = initial_norm[0]*halfway_norm[1] - initial_norm[1]*halfway_norm[0]
    
    # Return the quaternion
    return [w, x, y, z]

def estimage_gravity(acceleration, gyroscope):
     

while True:
	imu.readSensor()
	for i in range(10):
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime
          
		g_force = estimage_gravity([imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]], [imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]])

		# Project vec3d onto the plane orthogonal to orthogonal_vec
		projected_vec = calculate_ned_frame([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]], [g_force[0], g_force[1], g_force[2]])
        
		yaw_quaternion = calculate_quaternion_from_coordinates([1,0,0], projected_vec[1], projected_vec[2])
          
		yaw_euler = quaternion_to_euler(yaw_quaternion)

		sensorfusion.updateRollAndPitch(g_force[0], g_force[1], g_force[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], dt)

		comp_vel = speed_est.estimate_velocity(newTime, newTime-dt, g_force[0], g_force[1], g_force[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], sensorfusion.q)
          
		quaternion_y = euler_to_quaternion(math.radians(sensorfusion.roll), math.radians(sensorfusion.pitch), math.radians(yaw_euler[2]))
          
	if print_count == 20:
		print_count = 0

	print_count = print_count + 1
	time.sleep(0.001)