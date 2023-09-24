from math import pi
from typing import Tuple

import numpy as np

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

def npi_to_pi_angle(angle):
	"""
	Return the 'smaller version' of a radian angle.

	Ex: 3pi -> pi
	Ex: 1.5pi -> -0.5pi
	"""
	if angle > pi:
		return angle - 2*pi
	elif angle < -pi:
		return angle + 2*pi
	return angle

def quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.

	Input
		roll: (rotation around x-axis) angle in radians.
		pitch: (rotation around y-axis) angle in radians.
		yaw: (rotation around z-axis) angle in radians.

	Output
		qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
		np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
		np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
		np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
		np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]

def heading_to_quaternion(heading: float) -> Tuple[float, float, float, float]:
	return quaternion_from_euler(0, 0, heading)
