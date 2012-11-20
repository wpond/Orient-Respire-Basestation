import serial
from struct import *
import time

COMPORT = "COM15"
current_time = time.mktime(time.gmtime())

while 1:
	try:
		f = serial.Serial(COMPORT, baudrate=57600)
		break
	except serial.SerialException:
		print "failed to open serial, retrying..."
		time.sleep(1)

while(1):
	new_time = time.mktime(time.gmtime())
	if not new_time == current_time:
		current_time = new_time
		f.write(pack(">bixxxxxxxxxxxxxxxxxxxxxxxxxxx", 0, current_time))
