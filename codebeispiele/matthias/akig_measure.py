# AKIG 2020 - Matthias Rosa 1607526
# Messprogramm der Totalstation am Husky
# Publishes the measured Hz,V and D values

import serial
import time
import rospy
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import math

# Definition des ROS-Publishers und der Seriellen Schnittstelle fuer die Kommunikation mit dem Tachy
pub = rospy.Publisher('tachymeasure', geometry_msgs.msg.PointStamped, queue_size=1)
rospy.init_node("tachymeasure", anonymous=True)
tachy = serial.Serial('/dev/ttyUSB0', 9600)

# Klasse mit allen verwendeten GEOCOM Codes
class codes():
	drehen = 9027
	anziehlen = 17020
	messen = 2117
	stop = 2008

# Funktion fuer die Erstellung des Befehls an den Tachy
def request(string):
	tachy.write('\r\n{0}\r\n'.format(string).encode('ascii'))
	ans = answer()
	return ans

# Auslesen der Tachyantwort nach Durchfuehrung des Befehls
def answer():
	ans = tachy.readline()
	ans = ans.decode('ascii')
	ans = ans.replace('\r\n','')
	msg = ans.rsplit(':')
	data = msg[1]
	data = data.rsplit(',')
	return data

# Hauptfunktion, welche eine Liste an Fixpunkten anzielt, misst und die gemessenen Werte published
def station(pub):
	data = rospy.wait_for_message("/fixed_points", sensor_msgs.msg.PointCloud2)
	data = list(pc2.read_points(data))
	cmd = geometry_msgs.msg.PointStamped()

# Keine Abbruchbedingung!! Bei langer Liste an Punkten werden alle einzeln angezielt
	for i in range(len(data)):
		hz = data[i][3]
		v = data[i][4]

		drehen = '%R1Q,{0}:{1},{2}'.format(codes.drehen,hz,v)
		request(drehen)
		ausrichten = '%R1Q,{0}:'.format(codes.anziehlen)
		request(ausrichten)
		messen = '%R1Q,{0}:'.format(codes.messen)
		hzvd = request(messen)
		cmd.point.x  = float(hzvd[1])
		cmd.point.y  = float(hzvd[2])
		cmd.point.z  = float(hzvd[3])
		if cmd.point.z == 0:
			continue
		cmd.header = std_msgs.msg.Header()
		cmd.header.stamp = rospy.Time.now()
		cmd.header.frame_id = "/huskypose"
		pub.publish(cmd)
	stoppen = '%R1Q,{0}:0'.format(codes.stop)
	request(stoppen)

if __name__ =='__main__':
	try:
		station(pub)
	except rospy.ROSInterruptException:
		pass
