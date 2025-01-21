#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Opposite phase & Pivot turn:
   q    w    e 
   a    k    d
   z    s    c

In-Phase mode:
---------------------------
   Q    W    E
   A    K    D
   Z    S    C

k/K : stop

CTRL-C to quit

"""

moveBindings = {
		'w':(1,0,0,0),
		'e':(1,0,0,-1),
		'a':(0,0,0,1),
		'd':(0,0,0,-1),
		'q':(1,0,0,1),
		's':(-1,0,0,0),
		'c':(-1,0,0,1),
		'z':(-1,0,0,-1),
		'E':(1,-1,0,0),
		'W':(1,0,0,0),
		'A':(0,1,0,0),
		'D':(0,-1,0,0),
		'Q':(1,1,0,0),
		'S':(-1,0,0,0),
		'C':(-1,-1,0,0),
		'Z':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={ 
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	

	rclpy.init(args = args)
	node = rclpy.create_node('teleop_twist_keyboard')
		
	pub = node.create_publisher(Twist, '/kimm_4ws/cmd_vel', 	qos_profile=10)

	speed = 1.0
	turn = 1.0
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  
if __name__ == '__main__':
    main()
  