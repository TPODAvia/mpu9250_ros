import numpy as np
import math
    
class Integral:
    def __init__(self):
        self.__value = 0

    def integrate(self, x1, x2, y1, y2):
        self.__value += (y1 + y2) * (x2 - x1) * 0.5
        return self.__value

    def set_to_zero(self):
        self.__value = 0

class Speed_Estimation:
	
	def __init__(self):
		self.GRAVITY = 9.8
		self.samplesCount = 0
		self.velocityX = 0
		self.velocityY = 0
		self.velocityZ = 0

		# Threshold for number of samples after which device is considered stationary
		self.STATIC_SAMPLES_THRESHOLD = 6
		# Threshold value for acceleration after which device is considered stationary
		self.STATIC_ACCELERATION_THRESHOLD = 0.35
		# Threshold value for angular velocity after which device is considered stationary
		self.STATIC_ANGULAR_VELOCITY_THRESHOLD = 0.35

		self.intX = Integral()
		self.intY = Integral()
		self.intZ = Integral()
		self.changeStateTimesX = [[0, 0],[0, 0]]
		self.changeStateTimesY = [[0, 0],[0, 0]]
		self.changeStateTimesZ = [[0, 0],[0, 0]]
		
		self.firstControlNr = 0
		self.first = True
		self.previousState = True
		self.last_compensatedGravity = [0,0,0]
		self.now_compensatedGravity = [0,0,0]

	def estimate_velocity(self, time_now, time_previous, ax, ay, az, wx, wy, wz, q):

		if self.first:
			self.firstControlNr = time_previous
			self.first = False

		# if (time_now - self.firstControlNr) > 5 :
		if True:

			self.now_compensatedGravity = self.compensate_gravity([ax, ay, az], q)

			# Detecting static state
			if self.zeroVelocityUpdate(self.last_compensatedGravity, [wx, wy, wz]):

				self.velocityX = self.intX.integrate(time_previous, time_now, self.last_compensatedGravity[0], self.now_compensatedGravity[0])
				self.velocityY = self.intY.integrate(time_previous, time_now, self.last_compensatedGravity[1], self.now_compensatedGravity[1])
				self.velocityZ = self.intZ.integrate(time_previous, time_now, self.last_compensatedGravity[2], self.now_compensatedGravity[2])

				# Add velocity and time at the moment of movement start
				if not self.previousState:
					# self.changeStateTimes* = [[1, 2] <-- [3, 4]] becoming [[3, 4], [5, 6]]
					self.changeStateTimesX = [ [self.changeStateTimesX[1][0], self.changeStateTimesX[1][1]], \
							                   [time_previous,                  self.velocityX] ]
					
					self.changeStateTimesY = [ [self.changeStateTimesY[1][0], self.changeStateTimesY[1][1]], \
							                   [time_previous,                  self.velocityY] ]
					
					self.changeStateTimesZ = [ [self.changeStateTimesZ[1][0], self.changeStateTimesZ[1][1]], \
							                   [time_previous,                  self.velocityZ] ]
				self.previousState = True

			else:
				self.velocityX = self.intX.integrate(time_previous, time_now, self.last_compensatedGravity[0], self.now_compensatedGravity[0])
				self.velocityY = self.intY.integrate(time_previous, time_now, self.last_compensatedGravity[1], self.now_compensatedGravity[1])
				self.velocityZ = self.intZ.integrate(time_previous, time_now, self.last_compensatedGravity[2], self.now_compensatedGravity[2])

				# Add velocity and time at the moment of movement end
				if self.previousState:
					self.changeStateTimesX = [ [self.changeStateTimesX[1][0], self.changeStateTimesX[1][1]], \
							                   [time_previous,                  self.velocityX] ]
					
					self.changeStateTimesY = [ [self.changeStateTimesY[1][0], self.changeStateTimesY[1][1]], \
							                   [time_previous,                  self.velocityY] ]
					
					self.changeStateTimesZ = [ [self.changeStateTimesZ[1][0], self.changeStateTimesZ[1][1]], \
							                   [time_previous,                  self.velocityZ] ]
				self.previousState = False

		# Gravity Compensation in the past
		self.last_compensatedGravity = self.now_compensatedGravity

		# if not self.changeStateTimesX and not self.changeStateTimesY and not self.changeStateTimesZ:
		# 	self.changeStateTimesX.append((0, 0))
		# 	self.changeStateTimesY.append((0, 0))
		# 	self.changeStateTimesZ.append((0, 0))
		# elif len(self.changeStateTimesX) % 2 == 1:
		# 	self.changeStateTimesX.append((time_previous, self.velocityX))
		# 	self.changeStateTimesY.append((time_previous, self.velocityY))
		# 	self.changeStateTimesZ.append((time_previous, self.velocityZ))
			
		# return [self.velocityX, self.velocityY, self.velocityZ]
		print(self.now_compensatedGravity[0])
		return self.compensate_velocity_and_compute_displacement(self.velocityX, self.velocityY, self.velocityZ, time_now)

	def compensate_velocity_and_compute_displacement(self, velocityX, velocityY, velocityZ, time_now):

		xX = [self.changeStateTimesX[0][0], self.changeStateTimesX[1][0]]
		xY = [self.changeStateTimesY[0][0], self.changeStateTimesY[1][0]]
		xZ = [self.changeStateTimesZ[0][0], self.changeStateTimesZ[1][0]]

		yX = [self.changeStateTimesX[0][1], self.changeStateTimesX[0][1]]
		yY = [self.changeStateTimesY[0][1], self.changeStateTimesY[0][1]]
		yZ = [self.changeStateTimesZ[0][1], self.changeStateTimesZ[0][1]]


		compensated_velocity = [velocityX - self.linear_function(xX, yX, time_now),
                                velocityY - self.linear_function(xY, yY, time_now),
                                velocityZ - self.linear_function(xZ, yZ, time_now)]
		
		return compensated_velocity

	def zeroVelocityUpdate(self, acceleration, angularVelocity):
		if abs(acceleration[0]) <= self.STATIC_ACCELERATION_THRESHOLD and \
           abs(acceleration[1]) <= self.STATIC_ACCELERATION_THRESHOLD and \
           abs(acceleration[2]) <= self.STATIC_ACCELERATION_THRESHOLD and \
           abs(angularVelocity[0]) <= self.STATIC_ANGULAR_VELOCITY_THRESHOLD and \
           abs(angularVelocity[1]) <= self.STATIC_ANGULAR_VELOCITY_THRESHOLD and \
           abs(angularVelocity[2]) <= self.STATIC_ANGULAR_VELOCITY_THRESHOLD:
			self.samplesCount += 1
		else:
			self.samplesCount = 0

		return self.samplesCount >= self.STATIC_SAMPLES_THRESHOLD

	def compensate_gravity(self, acc, q):
		g = [0]*3
		g[0] = 2 * (q[1] * q[3] - q[0] * q[2]) * self.GRAVITY
		g[1] = 2 * (q[0] * q[1] + q[2] * q[3]) * self.GRAVITY
		g[2] = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * self.GRAVITY
		return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

	def linear_function(self, x, y, value):

		if (x[1] - x[0]) == 0:
			return 0
		
		a = (y[1] - y[0]) / (x[1] - x[0])
		b = y[0] - a * x[0]

		return a * value + b