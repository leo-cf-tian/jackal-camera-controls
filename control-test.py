#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Initialize the ROS node
rospy.init_node('jackal_keyboard_control')

# Create a publisher for sending velocity commands
pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

# Initialize the Twist message
twist = Twist()

# Define a function to read keyboard inputs
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main function for controlling the robot
def jackal_keyboard_control():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.loginfo("Use WASD keys to control the Jackal UGV. Press 'Ctrl + C' to quit.")

    while not rospy.is_shutdown():
        key = getKey()

        if key == 'w':
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -0.2
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = 1.0
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -1.0
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif key == '\x03':
            break

        pub.publish(twist)

    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        jackal_keyboard_control()
    except rospy.ROSInterruptException:
        pass
