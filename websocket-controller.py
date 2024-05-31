#!/usr/bin/env python

from collections import defaultdict
import rospy
from geometry_msgs.msg import Twist

from threading import Thread
import asyncio
import websockets
import json

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

keys = defaultdict(bool)

# Main function for controlling the robot
def jackal_keyboard_control():
    global keys

    while not rospy.is_shutdown():
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Jackal movement controls
        if keys["w"]:
            twist.linear.x = 0.5
        elif keys["s"]:
            twist.linear.x = -0.5
        
        if keys["a"]:
            twist.angular.z = 1.0
        elif keys["d"]:
            twist.angular.z = -1.0

        # Camera control
        if keys["down"]:
            cam_cmd.pitch = 60
        elif keys["up"]:
            cam_cmd.pitch = -60
        else:
            cam_cmd.pitch = pitch_now

        if keys["left"]:
            cam_cmd.yaw = 60
        elif keys["right"]:
            cam_cmd.yaw = -60
        else:
            cam_cmd.yaw = yaw_now

        cam_cmd.pitch = max(min(cam_cmd.pitch, 60), -60)
        cam_cmd.yaw = max(min(cam_cmd.yaw, 60), -60)

        pub.publish(twist)
        cam_pub.publish(cam_cmd)


async def handle_input(websocket, path):
    global keys

    try:
        async for message in websocket:
            data = json.loads(message)
            keys[data["key"]] = data["event"] == "down"
            print(keys)
    except websockets.exceptions.ConnectionClosedError:
        print("Client disconnected")

async def main():
    async with websockets.serve(handle_input, "192.168.0.159", 8765):
        print("WebSocket server started...")
        print("Use WASD keys to control the Jackal UGV. Use arrow keys to control the camera. Press 'Ctrl + C' to quit.")
        await asyncio.Future()

if __name__ == '__main__':
    try:
        t1 = Thread(target=jackal_keyboard_control)
        t2 = Thread(target=asyncio.run, args=(main(),))
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except rospy.ROSInterruptException:
        pass
