#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import keyboard
import math

import sys
import os
sys.path.insert(1, os.path.join(os.path.expanduser('~'), '/ros_ws/devel/lib/python3/dist-packages/pan_tilt_msgs'))

from pan_tilt_msgs.msg import PanTiltCmdDeg, PanTiltStatus

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

yaw_now, pitch_now = 0, 0
def update_current_cam_angle(yaw, pitch):
    global yaw_now, pitch_now
    yaw_now, pitch_now = yaw, pitch
cam_sub = rospy.Subscriber('/pan_tilt_status', PanTiltStatus, lambda data : update_current_cam_angle(data.yaw_now, data.pitch_now))

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
        if keyboard.is_pressed('down'):
            cam_cmd.pitch += 0.02
        elif keyboard.is_pressed('up'):
            cam_cmd.pitch -= 0.02
        else:
            cam_cmd.pitch = pitch_now

        if keyboard.is_pressed('left'):
            cam_cmd.yaw += 0.02
        elif keyboard.is_pressed('right'):
            cam_cmd.yaw -= 0.02
        else:
            cam_cmd.yaw = yaw_now

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