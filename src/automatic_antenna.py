#!/usr/bin/env python3

from math import atan2, radians
from threading import Lock

import pymap3d as pm

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Float64, Int32
from geographic_msgs.msg import GeoPoint

from helpers import clamp, npi_to_pi_angle, heading_to_quaternion
from compass import read_compass
from gps import GPS
from servo import Servo

### helpers ##################################################################

def pose_to_waypoint_heading(pose: Pose, waypoint: Point):
	# find the error in cartesian coordinates
	x_error = waypoint.x - pose.position.x
	y_error = waypoint.y - pose.position.y

	# calculate the heading that would point at the waypoint
	heading_to_waypoint = atan2(y_error, x_error)

	return npi_to_pi_angle(heading_to_waypoint)

### main #####################################################################

def main():
	AntennaControl().loop()

class AntennaControl:

	def __init__(self):

		rospy.init_node("antenna_control")

		### local variables ##################################################

		self.frequency = rospy.get_param("~frequency")
		self.servo_pos_tolerance = rospy.get_param("~servo_pos_tolerance")

		self.global_origin = None
		self.global_origin_lock = Lock()

		self.gps = None
		self.gps_module = GPS('/dev/ttyS0', timeout=2)  # TODO: rospy params
		self.gps_lock = Lock()

		self.pose = None
		self.pose_lock = Lock()

		self.servo_position = True
		self.servo = Servo(pwm_pin=12, freq=400, initial_pos=50, servo_pos_per_second=25)  # TODO: rospy params
		self.servo_position_lock = Lock()

		self.tracking_enabled = True
		self.tracking_enabled_lock = Lock()

		### calibrate servo ##################################################

		# sweep servo range
		self.set_servo_pos(self.servo.MIN_POS)
		self.min_servo_heading = read_compass()
		self.set_servo_pos(self.servo.MAX_POS)
		self.max_servo_heading = read_compass()
		servo_heading_range = self.min_servo_heading - self.max_servo_heading

		self.radians_heading_per_servo_pos = servo_heading_range / self.servo.POS_RANGE

		### connect to ROS ###################################################

		global_origin_topic = rospy.get_param("~global_origin_topic")
		rover_position_topic = rospy.get_param("~rover_position_topic")
		target_heading_topic = rospy.get_param("~target_heading_topic")
		target_servo_pos_topic = rospy.get_param("~target_servo_pos_topic")
		global_position_topic = rospy.get_param("~global_position_topic")
		local_position_topic = rospy.get_param("~local_position_topic")
		enable_tracking_service = rospy.get_param("~enable_tracking_service")

		self.global_origin_sub = rospy.Subscriber(global_origin_topic, GeoPoint, self.global_origin_callback)
		self.rover_position_sub = rospy.Subscriber(rover_position_topic, Pose, self.rover_position_callback)
		self.target_heading_sub = rospy.Subscriber(target_heading_topic, Float64, self.target_heading_callback)
		self.target_servo_pos_sub = rospy.Subscriber(target_servo_pos_topic, Int32, self.target_servo_pos_callback)
		self.global_position_pub = rospy.Publisher(global_position_topic, GeoPoint, queue_size=1)
		self.local_position_pub = rospy.Publisher(local_position_topic, Pose, queue_size=1)
		self.enable_tracking_srv = rospy.Service(enable_tracking_service, SetBool, self.enable_tracking_callback)

		### end init #########################################################

	### local functions ######################################################

	def update_pose(self):

		# update gps location
		try:
			lat, lon, h = self.gps_module.get_position()
		except IOError:
			rospy.logdebug("Error reading GPS position. Couldn't update Pose.")
			return
		with self.gps_lock:
			new_gps = GeoPoint()
			new_gps.latitude = lat
			new_gps.longitude = lon
			new_gps.altitude = h
			self.gps = new_gps

		if not self.global_origin:
			rospy.logdebug("No global origin. Couldn't update Pose.")
			return

		# convert gps location to ENU
		lat0 = self.global_origin.latitude
		lon0 = self.global_origin.longitude
		h0 = self.global_origin.altitude
		e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)

		# update pose
		heading = read_compass()
		w, x, y, z = heading_to_quaternion(heading)
		with self.pose_lock:
			self.pose.position = Point(e, n, u)
			self.pose.orientation = Quaternion(w, x, y, z)

	def point_toward(self, point: Point):
		if self.pose:
			heading = pose_to_waypoint_heading(self.pose, point)
			self.point_with_heading(heading)

	def point_with_heading(self, heading: float):
		"""Point to a heading specified in radians"""
		servo_position = self.heading_to_servo_pos(heading)
		self.set_servo_pos(servo_position)

	def heading_to_servo_pos(self, heading: float) -> float:
		heading = clamp(heading, self.min_servo_heading, self.max_servo_heading)
		degrees_from_far_left = (heading - self.min_servo_heading)
		servo_pos = degrees_from_far_left // self.radians_heading_per_servo_pos
		return servo_pos

	def set_servo_pos(self, raw_servo_position: int):
		servo_position = clamp(raw_servo_position, self.servo.MIN_POS, self.servo.MAX_POS)
		with self.servo_position_lock:

			# check if its worth moving the servo
			servo_delta = abs(servo_position - self.servo_position)
			if servo_delta < self.servo_pos_tolerance:
				return

			try:
				self.servo.set_position(servo_position)
			except ValueError as e:
				rospy.logwarn(e)
				return

	### callbacks ############################################################

	def global_origin_callback(self, gps_position: GeoPoint):
		with self.global_origin_lock:
			self.global_origin = gps_position

	def rover_position_callback(self, rover_pose: Pose):
		with self.tracking_enabled_lock:
			tracking_enabled = self.tracking_enabled

		if tracking_enabled:
			self.point_toward(rover_pose.position)

	def target_heading_callback(self, target_heading: Float64):
		"""Points to a heading specified in degrees (if tracking is disabled)"""
		with self.tracking_enabled_lock:
			tracking_enabled = self.tracking_enabled

		if not tracking_enabled:
			target_heading_rad = radians(target_heading.data)
			self.point_with_heading(target_heading_rad)

	def target_servo_pos_callback(self, target_servo_pos: Int32):
		"""Sets servo position (if tracking is disabled)"""
		with self.tracking_enabled_lock:
			tracking_enabled = self.tracking_enabled

		if not tracking_enabled:
			self.set_servo_pos(target_servo_pos.data)

	def enable_tracking_callback(self, bool: SetBoolRequest) -> SetBoolResponse:
		tracking_enabled = bool.data
		with self.tracking_enabled:
			self.tracking_enabled = tracking_enabled

		response = SetBoolResponse()
		response.success = True
		response.message = "Updated antenna control's tracking_enable status"
		return response

	### loop #################################################################

	def loop(self):
		rate = rospy.Rate(self.frequency)
		while not rospy.is_shutdown():

			self.update_pose()

			# publish antenna position and heading
			with self.gps_lock:
				if self.gps:
					self.global_position_pub.publish(self.gps)
			with self.pose_lock:
				if self.pose:
					self.local_position_pub.publish(self.pose)

			rate.sleep()

if __name__ == "__main__":
	main()
