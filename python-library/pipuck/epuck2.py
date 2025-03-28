from typing import Optional
import os
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
SPEAKER_SOUND = 4
LEDS_1357 = 5
LEDS_RGB_2 = 6
LEDS_RGB_4 = 9
LEDS_RGB_6 = 12
LEDS_RGB_8 = 15
SETTINGS = 18

# Battery
EPUCK_BATTERY_PATH = "/sys/bus/i2c/devices/11-0048/iio:device0/in_voltage0_raw"
EPUCK_BATTERY_PATH_LEGACY = "/sys/bus/i2c/drivers/ads1015/3-0048/in4_input"
AUX_BATTERY_PATH = "/sys/bus/i2c/devices/11-0048/iio:device0/in_voltage1_raw"
AUX_BATTERY_PATH_LEGACY = "/sys/bus/i2c/drivers/ads1015/3-0048/in5_input"
EPUCK_BATTERY_SCALE_PATH = "/sys/bus/i2c/devices/11-0048/iio:device0/in_voltage0_scale"
AUX_BATTERY_SCALE_PATH = "/sys/bus/i2c/devices/11-0048/iio:device0/in_voltage1_scale"
LEGACY_BATTERY_SCALE = 1.0
BATTERY_MIN_VOLTAGE = 3.3
BATTERY_MAX_VOLTAGE = 4.138
BATTERY_VOLTAGE_RANGE = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE


def int_to_2byte(value):
	# Ensure the value is within the range [-1024, 1024]
	if not (-1024 <= value <= 1024):
		raise ValueError("Value must be between -1024 and 1024")

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

		self.epuck_battery_path = ""
		self.aux_battery_path = ""
		self.epuck_scale_path = ""
		self.aux_scale_path = ""
		self.scale = 0.0
		self.voltage = 0.0
		self.raw_value = 0
		self.percentage = 0.0

		# Determine actual path to use for ADC driver (try iio, then hwmon)
		if os.path.exists(EPUCK_BATTERY_PATH):
			self.epuck_battery_path = EPUCK_BATTERY_PATH
			self.aux_battery_path = AUX_BATTERY_PATH
			self.epuck_scale_path = EPUCK_BATTERY_SCALE_PATH
			self.aux_scale_path = AUX_BATTERY_SCALE_PATH
		elif os.path.exists(EPUCK_BATTERY_PATH_LEGACY):
			self.epuck_battery_path = EPUCK_BATTERY_PATH_LEGACY
			self.aux_battery_path = AUX_BATTERY_PATH_LEGACY
			self.epuck_scale_path = None
			self.aux_scale_path = None
		else:
			print("Cannot read ADC path")

	def update_robot_sensors_and_actuators(self):
		checksum = 0
		for i in range(ACTUATORS_SIZE-1):
			checksum ^= self.actuators_data[i]
		self.actuators_data[ACTUATORS_SIZE-1] = checksum

		try:
			write = i2c_msg.write(ROB_ADDR, self.actuators_data)
			read = i2c_msg.read(ROB_ADDR, SENSORS_SIZE)
			self._bus.i2c_rdwr(write, read)
			self.sensors_data = list(read)
		except:
			sys.exit(1)

		# Verify the checksum (Longitudinal Redundancy Check) before interpreting the received sensors data.
		checksum = 0
		for i in range(SENSORS_SIZE-1):
			checksum ^= self.sensors_data[i]
		if(checksum != self.sensors_data[SENSORS_SIZE-1]):
			print("wrong checksum ({0:#x} != {0:#x})\r\n".format(self.sensors_data[ACTUATORS_SIZE-1], checksum))

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
		self.update_robot_sensors_and_actuators()

	def set_speaker_sound(self, sound):
		if(sound == 0 or sound == 1 or sound == 2):
			self.actuators_data[SPEAKER_SOUND] = sound
		else:
			print('[ERROR]: Speaker input must be [0, 1, 2]')

		self.update_robot_sensors_and_actuators()

	def set_leds(self, led1=0, led3=0, led5=0, led7=0):
		binary_str = f'{led7}{led5}{led3}{led1}' # Convert input to a string
		hex_str = f'0x{int(binary_str, 2):X}' # Format as hexadecimal string
		hex_value = int(hex_str, 16) # Convert string to hex
		self.actuators_data[LEDS_1357] = hex_value
		self.update_robot_sensors_and_actuators()

	def set_rgb_led(self, led, color):
		rgb_led = None
		if led == 2:
			rgb_led = LEDS_RGB_2
		elif led == 4:
			rgb_led = LEDS_RGB_4
		elif led == 6:
			rgb_led = LEDS_RGB_6
		elif led == 8:
			rgb_led = LEDS_RGB_8

		for i, c in enumerate(color):
			# Ensure the value is within the range [0, 100]
			if not (0 <= c <= 100):
				raise ValueError("Value must be between 0 and 100")

			self.actuators_data[rgb_led + i] = c
			self.update_robot_sensors_and_actuators()

	def set_settings(self, bit0, bit1, bit2):
		binary_str = f'{bit2}{bit1}{bit0}' # Convert input to a string
		hex_str = f'0x{int(binary_str, 2):X}' # Format as hexadecimal string'
		hex_value = int(hex_str, 16) # Convert string to hex
		self.actuators_data[SETTINGS] = hex_value
		self.update_robot_sensors_and_actuators()

	@property
	def battery(self):
		# Read e-puck battery
		if self.epuck_scale_path is not None:
			with open(self.epuck_scale_path, "r") as scale_file:
				scale = float(scale_file.read())
		else:
			scale = LEGACY_BATTERY_SCALE

		with open(self.epuck_battery_path, "r") as battery_file:
			raw_value = float(battery_file.read())
			voltage = round((raw_value * scale) / 500.0, 2)

		percentage = round((voltage - BATTERY_MIN_VOLTAGE) / BATTERY_VOLTAGE_RANGE * 100.0, 2)
		if percentage < 0.0:
			percentage = 0.0
		elif percentage > 100.0:
			percentage = 100.0

		# Read external battery
		if self.aux_scale_path is not None:
			with open(self.aux_scale_path, "r") as scale_file:
				scale = float(scale_file.read())
		else:
			scale = LEGACY_BATTERY_SCALE

		with open(self.aux_battery_path, "r") as battery_file:
			aux_raw_value = float(battery_file.read())
			aux_voltage = round((raw_value * scale) / 500.0, 2)

		aux_percentage = round((voltage - BATTERY_MIN_VOLTAGE) / BATTERY_VOLTAGE_RANGE * 100.0, 2)
		if aux_percentage < 0.0:
			aux_percentage = 0.0
		elif aux_percentage > 100.0:
			aux_percentage = 100.0

		return percentage, voltage, raw_value, aux_percentage, aux_voltage, aux_raw_value

	@property
	def ir_reflected(self):
		self.update_robot_sensors_and_actuators()
		for i in range(8):
			self.prox[i] = self.sensors_data[i*2+1]*256+self.sensors_data[i*2]
		return self.prox

	@property
	def ir_ambient(self):
		self.update_robot_sensors_and_actuators()
		for i in range(8):
			self.prox_amb[i] = self.sensors_data[16+i*2+1]*256+self.sensors_data[16+i*2]
		return self.prox_amb

	@property
	def microphone(self):
		self.update_robot_sensors_and_actuators()
		for i in range(4):
			self.mic[i] = self.sensors_data[32+i*2+1]*256+self.sensors_data[32+i*2]
		return self.mic

	@property
	def selector(self):
		self.update_robot_sensors_and_actuators()
		return self.sensors_data[40]&0x0F

	@property
	def button(self):
		self.update_robot_sensors_and_actuators()
		return self.sensors_data[40]>>4

	@property
	def motor_steps(self):
		self.update_robot_sensors_and_actuators()
		for i in range(2):
			self.mot_steps[i] = self.sensors_data[41+i*2+1]*256+self.sensors_data[41+i*2]
		return self.mot_steps[0], self.mot_steps[1]

	@property
	def tv_remote(self):
		self.update_robot_sensors_and_actuators()
		return self.sensors_data[45]
