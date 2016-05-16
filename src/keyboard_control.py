#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


import sys, select, termios, tty

msg = """
t : takeoff
l : landing
p : emergency stop/reset

anything else : stop

CTRL-C to quit
"""


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	takeoff_pub =rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 1)
	landing_pub =rospy.Publisher('/ardrone/land', Empty, queue_size = 1)
	stop_pub =rospy.Publisher('/ardrone/reset', Empty, queue_size = 1)

	rospy.init_node('teleop_twist_keyboard')

	try:
		print msg
		while(1):
			order = Empty()
			key = getKey()
			print key
			if key == 't':
				takeoff_pub.publish(order)
			elif key == 'l':
				landing_pub.publish(order)
			elif key == 'p':
				stop_pub.publish(order)
			elif key == 's':
				twist = Twist()
				twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
				vel_pub.publish(twist)
			else:
				if (key == '\x03'):
					break

	except rospy.ROSInterruptException:
        	pass

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		vel_pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


