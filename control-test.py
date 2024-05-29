#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from _PanTiltCmdDeg import PanTiltCmdDeg
import sys, select, termios, tty
import keyboard

# Initialize the ROS node
rospy.init_node('jackal_keyboard_control')

# Create a publisher for sending velocity commands
pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

# Create a publisher for sending camera angle commands
cam_pub = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=10)

# Initialize the Twist message
twist = Twist()

# Initialize the camera command message
cam_cmd = PanTiltCmdDeg()
cam_cmd.yaw = 0
cam_cmd.pitch = 0
cam_cmd.speed = 30


# Main function for controlling the robot
def jackal_keyboard_control():

    rospy.loginfo("Use WASD keys to control the Jackal UGV. Use arrow keys to control the camera. Press 'Ctrl + C' to quit.")

    while not rospy.is_shutdown():

        if keyboard.is_pressed('ctrl+c'):
            break

        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Jackal movement controls
        if keyboard.is_pressed('w'):
            twist.linear.x = 0.5
        elif keyboard.is_pressed('s'):
            twist.linear.x = -0.5
        
        if keyboard.is_pressed('a'):
            twist.angular.z = 1.0
        elif keyboard.is_pressed('d'):
            twist.angular.z = -1.0

        # Camera control
        if keyboard.is_pressed('k'):
            cam_cmd.pitch += 0.01
        elif keyboard.is_pressed('i'):
            cam_cmd.pitch -= 0.01

        if keyboard.is_pressed('j'):
            cam_cmd.yaw += 0.01
        elif keyboard.is_pressed('l'):
            cam_cmd.yaw -= 0.01

        cam_cmd.pitch = max(min(cam_cmd.pitch, 60), -60)
        cam_cmd.yaw = max(min(cam_cmd.yaw, 60), -60)

        pub.publish(twist)
        cam_pub.publish(cam_cmd)
    

    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    cam_pub.publish(cam_cmd)


if __name__ == '__main__':
    try:
        jackal_keyboard_control()
    except rospy.ROSInterruptException:
        pass