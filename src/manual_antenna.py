#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

from antenna_control.srv import SetPosition, SetPositionRequest, SetPositionResponse

from helpers import clamp
from servo import Servo

### main #####################################################################

def main():
	AntennaControl().loop()

class AntennaControl:

	def __init__(self):

		rospy.init_node("antenna_control")

		### local variables ##################################################

		pwm_pin = rospy.get_param("~pwm_pin")
		pwm_freq = rospy.get_param("~pwm_freq")
		initial_pos = rospy.get_param("~initial_pos")
		servo_pos_per_second = rospy.get_param("~servo_pos_per_second")
		self.servo = Servo(pwm_pin=pwm_pin, freq=pwm_freq, initial_pos=initial_pos, servo_pos_per_second=servo_pos_per_second)

		self.servo.MIN_POS = 0.0
		self.servo.MAX_POS = 100.0

		### connect to ROS ###################################################

		servo_position_topic = rospy.get_param("~servo_position_topic")
		servo_position_service = rospy.get_param("~servo_position_service")

		self.servo_position_pub = rospy.Publisher(servo_position_topic, Float32, queue_size=1)
		self.servo_position_srv = rospy.Service(servo_position_service, SetPosition, self.set_position)

		### end init #########################################################

	### callbacks ############################################################

	def set_position(self, raw_servo_position: Float32):
		servo_position = clamp(raw_servo_position.data, self.servo.MIN_POS, self.servo.MAX_POS)
		try:
			self.servo.set_position(servo_position)
		except ValueError as e:
			rospy.logwarn(e)

	### loop #################################################################

	def loop(self):
		rospy.spin()

if __name__ == "__main__":
	main()
