#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
AR Drone control by keyboard
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

For yaw control, hold down the shift key:
---------------------------
   J    K    L

; : up (+z)
. : down (-z)

T : takeoff
L : landing
P : emergency stop/reset

anything else : stop

1/2 : increase/decrease max speeds by 10%
3/4 : increase/decrease only linear speed by 10%
5/6 : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'q':(1,1,0,0),
		'w':(1,0,0,0),
		'e':(1,-1,0,0),
		'a':(0,1,0,0),
		'd':(0,-1,0,0),
		'z':(-1,1,0,0),
		'x':(-1,0,0,0),
		'c':(-1,-1,0,0),
		'A':(0,0,0,1),
		'D':(0,0,0,-1),	
		';':(0,0,1,0),
		'.':(0,0,-1,0),		
	       }

speedBindings={
		'1':(1.1,1.1),
		'2':(.9,.9),
		'3':(1.1,1),
		'4':(.9,1),
		'5':(1,1.1),
		'6':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	takeoff_pub =rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 1)
	landing_pub =rospy.Publisher('/ardrone/land', Empty, queue_size = 1)
	stop_pub =rospy.Publisher('/ardrone/reset', Empty, queue_size = 1)

	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			order = Empty()
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			elif key == 'T':
				takeoff_pub.publish(order)
			elif key == 'L':
				landing_pub.publish(order)
			elif key == 'P':
				stop_pub.publish(order)
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			vel_pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		vel_pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


