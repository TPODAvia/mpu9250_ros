#!/usr/bin/env python3
import rospy
import smbus
import time
import math
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import math
# import tf.transformations
# import tf2_ros
import json

# Initialize the sensor
yaw_compensate = True
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.setAccelRange("AccelRangeSelect8G")
imu.setGyroRange("GyroRangeSelect1000DPS")
imu.setLowPassFilterFrequency("AccelLowPassFilter5")
imu.loadCalibDataFromFile("/home/rover/gr_platform/ros/src/sensing/mpu9250_ros/config/calib.json")

with open('/home/rover/gr_platform/ros/src/sensing/mpu9250_ros/config/yaw_calib.json', 'r') as f:
    data = json.load(f)

# Initialize the filter
sensorfusion = madgwick.Madgwick(0.5)

# Shint in counteclockwise by 44 degrees
def rotate_angle(angle):
    shifted_angle = (angle + 44) % 360
    if shifted_angle > 180:
        shifted_angle -= 360
    elif shifted_angle < -180:
        shifted_angle += 360
    return shifted_angle

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

def talker():
    pub = rospy.Publisher('/mpu9250_ros/imu_data', Imu, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(100)  # 100hz
    currTime = rospy.get_time()
    
    # Initialize tf broadcaster
    # tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        newTime = rospy.get_time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'

        # Populate orientation data (convert roll, pitch, yaw to a quaternion) /// sensorfusion.q[0]
        if yaw_compensate:

            if (sensorfusion.yaw > -abs(data[0]) and sensorfusion.yaw < data[1]): 	# 0->90
                                    yaw_est = lookup(sensorfusion.yaw,-abs(data[0]),data[1],0,90)

            if (sensorfusion.yaw > data[1] and sensorfusion.yaw < data[2]): 		# 90->180
                                    yaw_est = lookup(sensorfusion.yaw,data[1],data[2],90,180)

            if (sensorfusion.yaw > data[2] and sensorfusion.yaw < data[3]): 		# 180->270
                                    yaw_est = lookup(sensorfusion.yaw,data[2],data[3],180,270)

            if (sensorfusion.yaw > data[3] and sensorfusion.yaw < 180):				# 270->360
                                    yaw_est = lookup(sensorfusion.yaw,data[3],abs(data[0]),270,360)

        #     quaternion = tf.transformations.quaternion_from_euler(math.radians(sensorfusion.roll), math.radians(sensorfusion.pitch), math.radians(yaw_est))
        # else:
        #     quaternion = tf.transformations.quaternion_from_euler(math.radians(sensorfusion.roll), math.radians(sensorfusion.pitch), math.radians(sensorfusion.yaw))
        rot_quat = euler_to_quaternion(math.radians(rotate_angle(sensorfusion.roll)), math.radians(sensorfusion.pitch), math.radians(sensorfusion.yaw))

        imu_msg.orientation.x = rot_quat[0]
        imu_msg.orientation.y = rot_quat[1]
        imu_msg.orientation.z = rot_quat[2]
        imu_msg.orientation.w = rot_quat[3]
        imu_msg.angular_velocity.x = imu.GyroVals[0]
        imu_msg.angular_velocity.y = imu.GyroVals[1]
        imu_msg.angular_velocity.z = imu.GyroVals[2]
        imu_msg.linear_acceleration.x = imu.AccelVals[0]
        imu_msg.linear_acceleration.y = imu.AccelVals[1]
        imu_msg.linear_acceleration.z = imu.AccelVals[2]
        # Publish the IMU message
        pub.publish(imu_msg)

        # # Broadcast the transform from 'odom' to 'imu_link'
        # t = TransformStamped()
        # t.header.stamp = imu_msg.header.stamp
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'imu_link'
        # # Assuming the IMU is mounted at the origin of the 'odom' frame (update if necessary)
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 0.0
        # t.transform.translation.z = 0.0
        # t.transform.rotation = imu_msg.orientation
        
        # # Send the transformation
        # tf_broadcaster.sendTransform(t)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass