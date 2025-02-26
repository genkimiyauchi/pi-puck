from typing import Optional
import sys
from .epuck import EPuck
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg
import struct

I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
RESET_PIN = 18
ACTUATORS_SIZE = (19+1) # Data + checksum.
SENSORS_SIZE = (46+1) # Data + checksum.

# Register addresses
LEFT_MOTOR_SPEED = 0
RIGHT_MOTOR_SPEED = 2


def int_to_2byte(value):
	# Ensure the value is within the range [-512, 512]
	if not (-512 <= value <= 512):
		raise ValueError("Value must be between -512 and 512")

	# Convert to a 16-bit signed integer
	packed_value = struct.pack('<h', value)  # '<h' means little-endian, short (2 bytes)
	
	# Return the two bytes as a list
	return list(packed_value)


class EPuck2(EPuck):
	"""Class for interfacing with an e-puck2 robot"""

	def __init__(self, i2c_bus: Optional[int] = None, i2c_address: Optional[int] = None):
		# super().__init__(i2c_bus, i2c_address)

		if i2c_bus is not None:
			self._bus = SMBus(i2c_bus)
		else:
			try:
				self._bus = SMBus(I2C_CHANNEL)
			except FileNotFoundError:
				self._bus = SMBus(LEGACY_I2C_CHANNEL)

		if i2c_address is not None:
			self._i2c_address = i2c_address
		else:
			self._i2c_address = ROB_ADDR

		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)

		self.actuators_data = bytearray([0] * ACTUATORS_SIZE)
		self.sensors_data = bytearray([0] * SENSORS_SIZE)
		self.prox = [0 for x in range(8)]
		self.prox_amb = [0 for x in range(8)]
		self.mic = [0 for x in range(4)]
		self.mot_steps = [0 for x in range(2)]

		try:
			self._bus = SMBus(I2C_CHANNEL)
		except:
			try:
				self._bus = SMBus(LEGACY_I2C_CHANNEL)
			except:
				print("Cannot open I2C device")
				sys.exit(1)

	def update_robot_sensors_and_actuators(self):
		try:
			write = i2c_msg.write(ROB_ADDR, self.actuators_data)
			read = i2c_msg.read(ROB_ADDR, SENSORS_SIZE)
			self._bus.i2c_rdwr(write, read)
			self.sensors_data = list(read)
		except:
			sys.exit(1)

	# def set_outer_leds_byte(self, leds):
	# 	self._write_data_8(OUTER_LEDS, leds)

	# def set_outer_leds(self, led0, led1, led2, led3, led4, led5, led6, led7):
	# 	data = 0x00
	# 	if led0:
	# 		data += 0x01
	# 	if led1:
	# 		data += 0x02
	# 	if led2:
	# 		data += 0x04
	# 	if led3:
	# 		data += 0x08
	# 	if led4:
	# 		data += 0x10
	# 	if led5:
	# 		data += 0x20
	# 	if led6:
	# 		data += 0x40
	# 	if led7:
	# 		data += 0x80
	# 	self.set_outer_leds_byte(data)

	# def set_inner_leds(self, front, body):
	# 	data = 0x00
	# 	if front:
	# 		data += 0x01
	# 	if body:
	# 		data += 0x02
	# 	self._write_data_8(INNER_LEDS, data)

	def set_left_motor_speed(self, speed):
		# self._write_data_16(LEFT_MOTOR_SPEED, int(speed))
		speed_bytes = int_to_2byte(int(speed))
		self.actuators_data[LEFT_MOTOR_SPEED] = speed_bytes[0]
		self.actuators_data[LEFT_MOTOR_SPEED+1] = speed_bytes[1]

	def set_right_motor_speed(self, speed):
		# self._write_data_16(RIGHT_MOTOR_SPEED, int(speed))
		speed_bytes = int_to_2byte(int(speed))
		self.actuators_data[RIGHT_MOTOR_SPEED] = speed_bytes[0]
		self.actuators_data[RIGHT_MOTOR_SPEED+1] = speed_bytes[1]

	def set_motor_speeds(self, speed_left, speed_right):
		self.set_left_motor_speed(speed_left)
		self.set_right_motor_speed(speed_right)

		checksum = 0
		for i in range(ACTUATORS_SIZE-1):
			checksum ^= self.actuators_data[i]		
		self.actuators_data[ACTUATORS_SIZE-1] = checksum
		
		self.update_robot_sensors_and_actuators()

	# @property
	# def left_motor_speed(self):
	# 	return self._read_data_16(LEFT_MOTOR_SPEED)

	# @property
	# def right_motor_speed(self):
	# 	return self._read_data_16(RIGHT_MOTOR_SPEED)

	# @property
	# def motor_speeds(self):
	# 	return self.left_motor_speed, self.right_motor_speed

	# @property
	# def left_motor_steps(self):
	# 	return self._read_data_16(LEFT_MOTOR_STEPS)

	# @property
	# def right_motor_steps(self):
	# 	return self._read_data_16(RIGHT_MOTOR_STEPS)

	# @property
	# def motor_steps(self):
	# 	return self.left_motor_steps, self.right_motor_steps

	# def enable_ir_sensors(self, enabled):
	# 	if enabled:
	# 		data = 0x01
	# 	else:
	# 		data = 0x00
	# 	self._write_data_8(IR_CONTROL, data)

	# def get_ir_reflected(self, sensor):
	# 	return self._read_data_16(IR_REFLECTED_BASE + sensor)

	# @property
	# def ir_reflected(self):
	# 	return [self.get_ir_reflected(i) for i in range(8)]

	# def get_ir_ambient(self, sensor):
	# 	return self._read_data_16(IR_AMBIENT_BASE + sensor)

	# @property
	# def ir_ambient(self):
	# 	return [self.get_ir_ambient(i) for i in range(8)]
