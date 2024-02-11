#!/usr/bin/env python3
import sys
import rospy
import smbus
import math
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from imusensor.filters import speed_estimation
import argparse
import math
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='My ROS Node with Argparse')
parser.add_argument('calib_file', type=str, default='~/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json', help='Calibration file')
args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

# Initialize the sensor
yaw_compensate = True
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.setAccelRange("AccelRangeSelect8G")
imu.setGyroRange("GyroRangeSelect1000DPS")
imu.setLowPassFilterFrequency("AccelLowPassFilter5")
# imu.loadCalibDataFromFile("/home/jetson/catkin_ws/src/OrcaRL/sensors/mpu9250_ros/config/calib.json")
imu.loadCalibDataFromFile(args.calib_file)

speed_est = speed_estimation.Speed_Estimation()

# Initialize the filter
sensorfusion = madgwick.Madgwick(0.5)
slow_vec = [0,0]
alpha = 0.001
acc_est = [0,0,0]
yaw_est = [0,1,0]
hysteresis_threshold =  270  # Define a threshold for the hysteresis

# Shint in counteclockwise by 44 degrees
def rotate_angle(angle):
    shifted_angle = (angle + 44) % 360
    if shifted_angle > 180:
        shifted_angle -= 360
    elif shifted_angle < -180:
        shifted_angle += 360
    return shifted_angle

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

def lookup(x, old_min, old_max, new_min, new_max):
    return ((new_max - new_min) * (x - old_min) / (old_max - old_min)) + new_min

def project_to_plane(input_vector, orthogonal_vector):
    # Find the projection matrix
    P = np.eye(3) - np.outer(orthogonal_vector, orthogonal_vector)
    
    # Project the input vector onto the plane
    projected_vector = P @ input_vector
    
    return projected_vector[:2] # Return only the first two components


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

def calculate_quaternion_from_coordinates(initial_vector, ortogonal_vector1, ortogonal_vector2, gravity):

    # print(f"{ortogonal_vector1} ::: {ortogonal_vector2}")
    if gravity > 0:
        desired_vector = [ortogonal_vector1[0]+ortogonal_vector2[1],ortogonal_vector1[1]-ortogonal_vector2[0],0]
    else:
        desired_vector = [ortogonal_vector1[0]-ortogonal_vector2[1],-ortogonal_vector1[1]-ortogonal_vector2[0],0]
    
    # Normalize the initial and final vectors
    initial_norm = initial_vector / np.linalg.norm(initial_vector)
    final_norm = desired_vector / np.linalg.norm(desired_vector)
    
    # Calculate the halfway vector
    halfway = final_norm + initial_norm

    # print(f"{halfway} ::: {final_norm}")
    
    # Normalize the halfway vector
    halfway_norm = halfway / np.linalg.norm(halfway)
    
    # Compute the quaternion components
    w = np.dot(initial_norm, halfway_norm)
    x = initial_norm[1]*halfway_norm[2] - initial_norm[2]*halfway_norm[1]
    y = initial_norm[2]*halfway_norm[0] - initial_norm[0]*halfway_norm[2]
    z = initial_norm[0]*halfway_norm[1] - initial_norm[1]*halfway_norm[0]
    
    # Return the quaternion
    return [w, x, y, z]

def estimage_gravity(accel, gyro, dt):

    a_norm_abs = abs(accel / np.linalg.norm(accel))

    if accel[0] > 0:
        k0 = 1
    else:
        k0 = -1

    if accel[1] > 0:
        k1 = 1
    else:
        k1 = -1

    if accel[2] > 0:
        k2 = 1
    else:
        k2 = -1
    
    # x: 1 (1->1) + 0 (0->1)
    # z: 0 (1->0) + 1 (1->1)
    acc_est[0] = (acc_est[0] - 8* (gyro[1]*k2*(1-a_norm_abs[1]) + gyro[2]*k1*(1-a_norm_abs[2])) *dt )*(1-alpha) + alpha * accel[0]
    acc_est[1] = (acc_est[1] + 8* (gyro[0]*k2*(1-a_norm_abs[0]) + gyro[2]*k0*(1-a_norm_abs[2])) *dt )*(1-alpha) + alpha * accel[1]
    acc_est[2] = (acc_est[2] + 8* (gyro[0]*k1*(1-a_norm_abs[0]) + gyro[1]*k0*(1-a_norm_abs[1])) *dt )*(1-alpha) + alpha * accel[2]

    return acc_est

def estimage_yaw(yaw, gyro_z, dt):

    yaw_est[0] = (yaw_est[0] - 70* gyro_z *dt )*(1-0.05) + 0.05 * yaw

    # Hysteresis correction
    if abs(yaw - yaw_est[0]) > hysteresis_threshold:
        if yaw_est[1] == 1:
            yaw_est[1] == 0
            yaw_est[0] = yaw

        # Correct the yaw estimation if the difference exceeds the threshold
        if yaw >  0 and yaw_est[0] < -160:
                yaw_est[0] +=  360
        elif yaw <  0 and yaw_est[0] >  160:
                yaw_est[0] -=  360

    if abs(yaw - yaw_est[0]) > 180:
        yaw_est[2] += 1
        if yaw_est[2] > 2:
            yaw_est[2] = 0
            yaw_est[0] = yaw
    else:
        yaw_est[2] = 0

    if yaw_est[0] > 180:
        return 180
    elif yaw_est[0] < -180:
        return -180
    else:
        return yaw_est[0]

def cancel_z_axis_rotation(quat):
    """
    Given a quaternion, this function cancels out the Z-axis rotation and returns the new quaternion.

    :param quat: A numpy array representing the original quaternion [w, x, y, z]
    :return: A numpy array representing the new quaternion with Z-axis rotation canceled out [w, x, y, z]
    """
    # Convert the quaternion to Euler angles
    rotation = R.from_quat(quat)
    euler_angles = rotation.as_euler('zyx', degrees=False)  # 'zyx' is for roll, pitch, yaw

    # Set the Z component of the Euler angles to  0
    euler_angles[2] =  0

    # Convert the modified Euler angles back to a quaternion
    new_rotation = R.from_euler('zyx', euler_angles, degrees=False)
    new_quat = new_rotation.as_quat()

    return new_quat

def quaternion_to_yaw_quaternion(input_quat):
    # Assuming input_quat is in the form [w, x, y, z]
    # Calculate the yaw angle from the input quaternion
    # yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    yaw_angle = math.atan2(2.0 * (input_quat[1] * input_quat[2] + input_quat[0] * input_quat[3]),  
                            -1+2.0 * (input_quat[0] * input_quat[0] + input_quat[1] * input_quat[1]))
    
    # Convert the yaw angle to a quaternion
    half_yaw =  - yaw_angle /  2.0

    return [math.cos(half_yaw),  0, 0, math.sin(half_yaw)]
    # return [1,  0, 0,  0]

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.

    :param q1: A  4-element array containing the first quaternion (q01, q11, q21, q31)
    :param q2: A  4-element array containing the second quaternion (q02, q12, q22, q32)

    :return: A  4-element array containing the final quaternion (q03, q13, q23, q33)
    """
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2

    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return [w, x, y, z]

def talker():

    pub = rospy.Publisher('/mpu9250_ros/imu_data', Imu, queue_size=10)
    vel_pub = rospy.Publisher('/mpu9250_ros/imu_vel', Twist, queue_size=10)

    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(30)  # 30hz
    currTime = rospy.get_time()
    
    # Initialize tf broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        imu.readSensor()
        for i in range(10):
            newTime = rospy.get_time()
            dt = newTime - currTime
            currTime = newTime

            # g_force = estimage_gravity([imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]], [imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]])
            projected_vec = calculate_ned_frame([imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]], [imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]])
            yaw_quaternion = calculate_quaternion_from_coordinates([1,0,0], projected_vec[1], projected_vec[2], imu.AccelVals[2])
            yaw_euler = quaternion_to_euler(yaw_quaternion)
            sensorfusion.updateRollAndPitch(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], dt)
            comp_vel = speed_est.estimate_velocity(newTime, newTime-dt, imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], sensorfusion.q)
            est_yaw = estimage_yaw(yaw_euler[2], imu.GyroVals[2], dt)

        quat0 = euler_to_quaternion(0, 0, -math.radians(est_yaw))
        quat1 = quaternion_to_yaw_quaternion(sensorfusion.q)
        quat2 = quaternion_multiply(quat1, sensorfusion.q)
        quad3 = quaternion_multiply(quat0, quat2)

        # print(f"{sensorfusion.roll} {sensorfusion.pitch} {est_yaw}")

        velo_msg = Twist()
        velo_msg.linear.x = comp_vel[0]
        velo_msg.linear.y = comp_vel[1]
        velo_msg.linear.z = comp_vel[2]
        velo_msg.angular.x = imu.GyroVals[0]
        velo_msg.angular.y = imu.GyroVals[1]
        velo_msg.angular.z = imu.GyroVals[2]
        vel_pub.publish(velo_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation.x = quad3[1]
        imu_msg.orientation.y = quad3[2]
        imu_msg.orientation.z = quad3[3]
        imu_msg.orientation.w = quad3[0]
        imu_msg.angular_velocity.x = imu.GyroVals[0]
        imu_msg.angular_velocity.y = imu.GyroVals[1]
        imu_msg.angular_velocity.z = imu.GyroVals[2]
        imu_msg.linear_acceleration.x = imu.AccelVals[0]
        imu_msg.linear_acceleration.y = imu.AccelVals[1]
        imu_msg.linear_acceleration.z = imu.AccelVals[2]
        # Publish the IMU message
        pub.publish(imu_msg)

        t = TransformStamped()
        t.header.stamp = imu_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = imu_msg.orientation
        
        # Send the transformation
        tf_broadcaster.sendTransform(t)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass