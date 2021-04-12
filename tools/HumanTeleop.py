#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Twist
#from hip_msgs.msg import Pose 
from hip_msgs.msg import detection 
from hip_msgs.msg import detections

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   7    8    9
   4    5    6
   1    2    3
/ 	: rotate counterclockwise
* 	: rotate clockwise
f/v : increase/decrease altitude
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop
CTRL-C to quit
"""

moveBindings = {
		'1':(-0.7,0.7,0),
		'2':(-1,0,0),
		'3':(-0.7,-0.7,0),
		'4':(0,1,0),
		'6':(0,-1,0),
		'7':(0.7,0.7,0),
		'8':(1,0,0),
		'9':(0.7,-0.7,0),
		'/':(0,0,1),
		'*':(0,0,-1)
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

posX = 0.0
posY = 0.0
posTheta = 0.0
dt = 0.1

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .1
turn = .2
e = "problem"

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/Jetson/cameraDetections', detections, queue_size=10)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	th = 0
	status = 0
	cnt = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()

			if key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					y = moveBindings[key][1]
					th = moveBindings[key][2]
				else:
					x = 0
					y = 0
					th = 0
					if (key == '\x03'):
						break

				posX = posX + x*speed*dt;
				posY = posY + y*speed*dt;
				posTheta = posTheta + th*turn*dt;

				#twist = Twist()
				#twist.linear.x = x*speed; twist.linear.z = 0; twist.linear.y = y*speed
				#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				Detection = detection()
				Detection.x = posX; Detection.y = posY; Detection.z = 0.2; Detection.p = 0;
#				pose.linear_velocity = math.sqrt((x*speed)*(x*speed) + (y*speed)*(y*speed)); pose.angular_velocity = th*turn;
				Detection2 = detection()
				Detection2.x = posX + 0.5; Detection2.y = posY + 0.5; Detection2.z = 0.2; Detection2.p = 0;

				detectionsOut = detections()
				detectionsOut.detections = [Detection, Detection2]
				detectionsOut.header.seq = cnt;
				detectionsOut.header.frame_id = "/Jetson"
				detectionsOut.header.stamp = rospy.Time.now()
				cnt = cnt + 1

				pub.publish(detectionsOut)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
#		pub.publish(twist)
#		detection = detection()
#		detection.x = 0; detection.y = 0; detection.theta = 0
#		detections = detections()
#		detections.detections = [detection]
#		detections.header.seq = cnt;
#		detections.header.frame_id = "/Jetson"
#		detections.header.stamp = rospy.Time.now()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
