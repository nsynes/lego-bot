from approxeng.input.selectbinder import ControllerResource
from gpiozero import Robot
from gpiozero import LED, DistanceSensor

import math
import random
import RPi.GPIO as GPIO
import time



SERVO_PIN_L = 17
SERVO_PIN_R = 18
LED_PIN = 27
TRIGGER_PIN = 22
ECHO_PIN = 24



class RobotStopException(Exception):
    """
    The simplest possible subclass of Exception, we'll raise this if we want to stop the robot
    for any reason. Creating a custom exception like this makes the code more readable later.
    """
    pass


class LegoBot(Robot):
	"""

	"""
	def __init__(self):
		super(LegoBot, self).__init__(
			left=('BOARD21', 'BOARD19'), right=('BOARD26', 'BOARD24')
		)
		
		self.init_led()
		self.init_servos()
		self.init_distance_sensor()
		
		self.buttons_held = set()
		self.auto_mode = False
		self.auto_mode_start_time = None
		self.light_toggle_on = False
		
		# Distance (cm) to start obstacle avoidance
		self.too_close = 20.0
				
		
	def init_led(self):
		"""
		Initialise LED.
		"""
		self.light = LED(LED_PIN)
		
	
	def init_servos(self):
		"""
		Initialise servo motors.
		
		Duty cycle notes:
		0 is no rotation
		2 is fastest clockwise
		6 is slowest clockwise
		7 is no rotation
		8 is slowest anti-clockwise
		12 is fastest anti-clockwise
		"""
		
		# Identify pins by GPIO number
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(SERVO_PIN_L, GPIO.OUT)
		GPIO.setup(SERVO_PIN_R, GPIO.OUT)
		
		# 50 Hz PWM signal
		self.left_servo = GPIO.PWM(SERVO_PIN_L, 50)
		self.right_servo = GPIO.PWM(SERVO_PIN_R, 50)

		# Servo initialization
		self.left_servo.start(0)
		self.right_servo.start(0)
		
		self.clockwise_duty_cycle = 6
		self.anticlockwise_duty_cycle = 8
		
		
	def init_distance_sensor(self):
		"""
		Initialise ultrasonic distance sensor
		"""
		
		self.sensor = DistanceSensor(echo=ECHO_PIN, trigger=TRIGGER_PIN)
	

	def set_speeds(self, power_left, power_right):
		"""
		Set speeds of left and right motors
		"""

		power_left = (-1 * power_left) / 100
		power_right = (-1 * power_right) / 100


		# If power less than 0, reverse motor direction
		if power_left < 0:
			self.left_motor.backward(-power_left)
		else:
			self.left_motor.forward(power_left)

		if power_right < 0:
			self.right_motor.backward(-power_right)
		else:
			self.right_motor.forward(power_right)


	def stop_motors(self):
		"""
		As we have an motor hat, stop the motors using their motors call
		"""
		# Turn both motors off
		self.left_motor.stop()
		self.right_motor.stop()
		
	
	def random_turn(self):
		"""
		Reverse, the rotate a random direction.
		"""
		
		reverse_time = 0.5
		
		# Randomly choose left or right
		motor_direction = [-40, 40]
		random.shuffle(motor_direction)
		
		turn_time = random.uniform(0.75, 2)

		# Back off a little
		self.set_speeds(-20, -20)
		time.sleep(reverse_time)
		self.stop()
		
		# Turn random direction
		self.set_speeds(*motor_direction)
		time.sleep(turn_time)
		self.set_speeds(0 ,0)
		
		
	def is_near_obstacle(self):
		"""
		Check if obstacle is too close.
		"""

		distance = self.sensor.distance * 100
		print(distance, end="\r", flush=True)

		if distance < self.too_close:
			return True
		else:
			return False
		
		
	def light_on(self):
		self.light.on()
		
		
	def light_off(self):
		self.light.off()
		
		
	def rotate_servo_clockwise(self, side):
		"""
		Rotate servo motor clockwise.
		`side` specifies "left" or "right" servo motor.
		"""
		duty_cycle = self.clockwise_duty_cycle
		if side == "left":
			self.left_servo.ChangeDutyCycle(duty_cycle)
		elif side == "right":
			self.right_servo.ChangeDutyCycle(duty_cycle)
		
	
	def rotate_servo_anticlockwise(self, side):
		"""
		Rotate servo motor anti-clockwise.
		`side` specifies "left" or "right" servo motor.
		"""
		duty_cycle = self.anticlockwise_duty_cycle
		if side == "left":
			self.left_servo.ChangeDutyCycle(duty_cycle)
		elif side == "right":
			self.right_servo.ChangeDutyCycle(duty_cycle)
			
	
	def stop_servo(self, side):
		"""
		Stop servo motor.
		`side` specifies "left" or "right" servo motor.
		"""
		if side == "left":
			self.left_servo.ChangeDutyCycle(0)
		elif side == "right":
			self.right_servo.ChangeDutyCycle(0)
			
	
	def run_manual_mode(self, x, y):
		"""
		Given x, y, check held buttons and operate robot via joystick.
		"""
		
		
		# Movement
		if "cross" in self.buttons_held:
			power_l_r = self.mixer(x, y)
			if power_l_r == [0, 0]:
				power_l_r = [20, 20]
		else:
			power_l_r = [0, 0]
		self.set_speeds(*power_l_r)
		
		# Lights
		self.light_on() if self.light_toggle_on else self.light_off()
		
		# Servo motors
		if "l1" in self.buttons_held and "l2" not in self.buttons_held:
			self.rotate_servo_anticlockwise("left")
		elif "l2" in self.buttons_held:
			self.rotate_servo_clockwise("left")
		else:
			self.stop_servo("left")
		
		if "r1" in self.buttons_held and "r2" not in self.buttons_held:
			self.rotate_servo_anticlockwise("right")
		elif "r2" in self.buttons_held:
			self.rotate_servo_clockwise("right")
		else:
			self.stop_servo("right")  
			
			
	def run_auto_mode(self):
		"""
		Run auto mode, where robot moves around and avoids any obstacles it detects.
		"""
		
		time_now = time.time()
		if math.floor(time_now - self.auto_mode_start_time)  % 2 == 0:
			self.light_on()
		else:
			self.light_off()

		if self.is_near_obstacle():
			self.set_speeds(0, 0)
			self.random_turn()
		else:
			self.set_speeds(20, 20)
			
			
	def track_buttons(self, joystick):
		"""
		Keep track of what buttons are currently held
		"""
		
		joystick.check_presses()
		
		if joystick.has_presses:
			self.buttons_held.update(joystick.presses)
		if joystick.has_releases:
			for released_button in joystick.releases:
				self.buttons_held.discard(released_button)
				
		if "select" in joystick.presses:
			self.auto_mode = not self.auto_mode
			self.auto_mode_start_time = time.time()
			print("auto mode:", self.auto_mode)
			
		if "circle" in joystick.presses:
			self.light_toggle_on = not self.light_toggle_on

		
	@staticmethod
	def mixer(yaw, throttle, max_power=100):
		"""
		Mix a pair of joystick axes, returning a pair of wheel speeds.
		yaw: axis value, from -1.0 to 1.0
		throttle: axis value, ranges from -1.0 to 1.0
		max_power: maximum power/speed returned from mixer
		
		Returns a tuple of integer values for left and right motor power
		"""
		left = throttle + yaw
		right = throttle - yaw
		scale = float(max_power) / max(1, abs(left), abs(right))

		return [int(left * scale), int(right * scale)]




robot = LegoBot()


try:
	while True:
		try:
			# Try to connect to an available controller
			with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
			
				print("Controlloer methods:", dir(joystick))
				print("Button names:", joystick.controls)
				
				while joystick.connected:
					
					robot.track_buttons(joystick)

					if robot.auto_mode:
						robot.run_auto_mode()
					else:
						x, y = joystick['l']
						robot.run_manual_mode(x, y)	
						
						if 'home' in joystick.presses:
							raise RobotStopException()
		except IOError:
			print('No controller found yet')
			time.sleep(1)
except RobotStopException:
    robot.stop_motors()





