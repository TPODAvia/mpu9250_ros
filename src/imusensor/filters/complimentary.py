


class Complimentary():
	"""
	Complimentary filter is a basic filter for sensor fusion

	The class fuses the roll, pitch and yaw from accelrometer
	and magneotmeter with gyroscope. 
	This is very basic. Even I don't know why I coded this.
	Not writing detailed doc for this as it is straightforward.

	"""
	def __init__(self, gain = 0.5):
		
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.gain = gain
		self.counterRoll = 0
		self.counterPitch = 0

	def setRoll(self, roll):
		self.roll = roll

	def setPitch(self, pitch):
		self.pitch = pitch

	def setYaw(self, yaw):
		self.yaw = yaw

	def setGain(self, gain):
		self.gain = gain

	def updateRollPitchYaw(self, measuredRoll, measuredPitch, measuredYaw, gx, gy, gz, dt):
		self.updateRollAndPitch(measuredRoll, measuredPitch, gx, gy, dt)
		self.yaw = self.update(self.yaw, measuredYaw, gz, dt)


	def updateRollAndPitch(self, measuredRoll, measuredPitch, gx, gy, dt):

		if measuredRoll < -160 or measuredRoll > 160:
			self.roll = measuredRoll
			self.counterRoll = 0
		if self.counterRoll < 20:
			self.roll = measuredRoll

		if measuredPitch < -160 or measuredPitch > 160:
			self.pitch = measuredPitch
			self.counterPitch = 0
		if self.counterPitch < 20:
			self.roll = measuredRoll

		self.counterRoll +=1
		self.counterPitch += 1
		self.roll = self.update(self.roll, measuredRoll, gx, dt)
		self.pitch = self.update(self.pitch, measuredPitch, gy, dt)

	def update(self, angle, measuredAngle, angularVelocity, dt):

		newAngle = (angle + 50*angularVelocity*dt)*(1 - self.gain)  + self.gain*(measuredAngle)
		return newAngle
