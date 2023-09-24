import serial
from time import time
from typing import Tuple

### constants ################################################################

FEET_PER_METER = 3.28084

# prefix in NMEA data string of GPGGA data
GPGGA_PREFIX = "$GPGGA,"

### conversion functions #####################################################

def degrees_from_nmea(nmea_str: str) -> float:
	"""
	The format for NMEA coordinates is (d)ddmm.mmmm
	d=degrees and m=minutes
	There are 60 minutes in a degree so divide the minutes by 60 and add that to the degrees.

	Ex: Latitude = 35.15 N
	35.15/60 = .5858 N

	Ex: Longitude = 12849.52 E,
	128 + 49.52/60 = 128.825333 E
	"""

	nmea_fl = float(nmea_str)
	ddd, mm_mmmm = divmod(nmea_fl, 100)
	whole_degrees = ddd
	decimal_degrees = mm_mmmm / 60
	degrees = whole_degrees + decimal_degrees
	return round(degrees, 4)

def gps_from_gpgga(gpgga_str: str) -> Tuple[float, float, float]:
	# extract NMEA data from GPGGA string
	gpgga_list = gpgga_str.split(',')
	nmea_latitude = gpgga_list[1]
	nmea_longitude = gpgga_list[3]
	nmea_altitude = gpgga_list[6]

	# convert NMEA data
	latitude = degrees_from_nmea(nmea_latitude)
	longitude = degrees_from_nmea(nmea_longitude)
	altitude = float(nmea_altitude) * FEET_PER_METER

	return Tuple[latitude, longitude, altitude]

### GPS class ################################################################

class GPS:
	timeout_msg = 'Timed out waiting for GPGGA data.'

	def __init__(self, device_path: str, timeout: float = None):
		self.ser = serial.Serial(port=device_path, timeout=timeout)
		self.timeout = timeout
		self.request_start = time()

	def _timed_out(self):
		if self.timeout:
			if time() - self.request_start > self.timeout:
				return True
		return False

	def get_position(self) -> Tuple[float, float, float]:
		self.request_start = time()

		# clear input buffer, to ensure that the next reading is the
		# current position
		self.ser.reset_input_buffer()

		try:
			while not self._timed_out():
				nmea_str = self.ser.readline().decode()
				gpgga_index = nmea_str.find(GPGGA_PREFIX)
				if (gpgga_index > 0):
					gpgga_str = nmea_str[gpgga_index:]
					return gps_from_gpgga(gpgga_str)

		except serial.SerialTimeoutException:
			raise TimeoutError(self.timeout_msg)

		raise TimeoutError(self.timeout_msg)

	def close(self):
		if self.ser.is_open():
			self.ser.close()

	def __exit__(self):
		self.close()

### example usage ############################################################

if __name__ == '__main__':
	gps = GPS('/dev/ttyS0', timeout=2)
	while True:
		try:
			lat, lon, alt = gps.get_position()
			print(f'latitude: {lat}, longitude: {lon}, altitude: {alt}')
		except (TimeoutError, serial.SerialException):
			print('error reading GPS position')
