import math
import rospy

from i2clibraries import i2c_hmc5883l
import geomag

from gps import GPS

def read_compass() -> float:

	gps_module = GPS('/dev/ttyS0', timeout=2)  # TODO: rospy params
	
	try:
		# Get gps location
		lat, lon, h = gps_module.get_position()

		# Get the angle between magnetic north and true north for the current gps position
		declination = geomag.declination(lat, lon, h)

		# Initialize compass module listening on I2C Port 1
		hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)

		# Gather heading angle data
		hmc5883l.setContinuousMode()
		hmc5883l.setDeclinationFromDeclination(declination)
		degrees = hmc5883l.getHeading()[0]

		return math.radians(degrees)
	
	except IOError:
		rospy.logdebug("Error reading GPS position. Failed to get heading angle from compass.")
		return