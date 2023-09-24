from time import sleep

import RPi.GPIO as gpio

def duty_from_pos(pos, freq):
	# convert val where 0 is retracted, 100 is extended
	# into 0 - 100 where 0 is 0% duty cycle and 100 is 100% duty
	min_ms = 0.9
	max_ms = 2.0
	range_ms = max_ms - min_ms
	duty_ms = 0.9 + (pos / 100) * range_ms
	period_ms = (1000 / freq)
	duty = duty_ms / period_ms * 100
	return duty

class Servo:
	MIN_POS = 0
	MAX_POS = 100
	POS_RANGE = 100

	def __init__(self, pwm_pin, freq, initial_pos, servo_pos_per_second):
		self.freq = freq
		self.pwm_pin = pwm_pin

		self.servo_pos_per_second = servo_pos_per_second
		self.max_sleep = self.POS_RANGE / self.servo_pos_per_second

		gpio.setmode(gpio.BOARD)
		gpio.setup(self.pwm_pin, gpio.OUT)
		self.servo_pwm = gpio.PWM(self.pwm_pin, self.freq)
				
		initial_duty = duty_from_pos(initial_pos, self.freq)
		self.servo_pwm.start(initial_duty)
		sleep(self.max_sleep)
		self.servo_pwm.ChangeDutyCycle(0)
		self.last_pos = initial_pos

	def set_position(self, pos):
		if not (0 <= pos <= 100):
			raise ValueError("Position must be in range 0-100")

		duty = duty_from_pos(pos, self.freq)
		self.servo_pwm.ChangeDutyCycle(duty)

		sleep_time = (abs(pos - self.last_pos) / self.servo_pos_per_second) if self.last_pos else self.max_sleep
		sleep(sleep_time)
		self.servo_pwm.ChangeDutyCycle(0)

		self.last_pos = pos

	def stop(self):
		gpio.cleanup()
		self.servo_pwm.stop()

if __name__ == '__main__':

	import signal

	##############################################################################

	PWM_PIN = 12
	FREQ = 400
	INITIAL_POS = 40
	SERVO_POS_PER_SECOND = 25

	##############################################################################

	# gracefully exit after capturing Ctrl-C
	def quit(signal, frame):
		print('Stopping...')
		servo.stop()
		exit(0)

	signal.signal(signal.SIGINT, quit)
	signal.signal(signal.SIGTERM, quit)

	##############################################################################

	# tell user how to use the program
	print('Type a number 0-100. This represents the position of the servo.')
	print()
	print(f'Setting servo to initial position {INITIAL_POS}...')
	servo = Servo(PWM_PIN, FREQ, INITIAL_POS, SERVO_POS_PER_SECOND)
	print(f'Done initializing servo')

	# main loop
	while True:

		# prompt user for a servo position
		value = input('> ')

		try:
			position = int(value)
			if not 0 <= position <= 100:
				print('Position must be an integer 0-100')
			else:
				print(f'Setting servo to position {position}...')
				servo.set_position(position)
				print(f'Done moving servo')

		except ValueError:
			print('Position must be an integer 0-100')
