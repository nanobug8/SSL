#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *


vision = SSL_DetectionFrame()

def vision_callback(data):
    global vision
    vision = data
    print(data.camera_id)


if __name__ == '__main__':
    rospy.init_node("stage_controller_node", anonymous = False)

    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)

    publisher = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size = 10)


    rate = rospy.Rate(10)
    msg = SSL()
    state = 0

    ball_x = 0
    ball_y = 0
    robot_x = 0
    robot_y = 0
    robot_ang = 0
    angle = 0
    dist = 1000

    angle_tol = 0.12

    state = 0

    while not rospy.is_shutdown():
        if (len(vision.balls) > 0):
            ball_x = vision.balls[0].x
            ball_y = vision.balls[0].y
            # print("Ball ", vision.balls[0].x, vision.balls[0].y)
            # print("\n")


        if (len(vision.robots_blue) > 0):
            robot_x = vision.robots_blue[0].x
            robot_y = vision.robots_blue[0].y
            robot_ang = vision.robots_blue[0].orientation
            # print("Robot ", vision.robots_blue[0].x, vision.robots_blue[0].y)
            # print("\n")

        if np.abs(ball_y - robot_y) > 0:
            angle = np.arctan2((ball_y - robot_y), (ball_x - robot_x))
            dist = np.sqrt(np.abs(ball_x - robot_x)**2 + np.abs(ball_y - robot_y)**2)

        # print(robot_ang)
        # print(angle)
        # print(angle, robot_ang, angle - robot_ang)

        if state == 0:

            print("state ", state)
            msg.dribbler = True
            msg.kicker = False
            
            # print(angle - robot_ang)
            if (angle - robot_ang > angle_tol):
                msg.cmd_vel.angular.z = 0.7
            elif (angle - robot_ang < - angle_tol):
                msg.cmd_vel.angular.z = - 0.7
            else:
                msg.cmd_vel.angular.z = 0.0

            # print(dist)
            if dist > 120:
                msg.cmd_vel.linear.x = 0.3 * np.cos(angle - robot_ang)
                msg.cmd_vel.linear.y = 0.3 * np.sin(angle - robot_ang)
            else:
                msg.cmd_vel.linear.x = 0.0 * np.cos(angle - robot_ang)
                msg.cmd_vel.linear.y = 0.0 * np.sin(angle - robot_ang)
                if msg.cmd_vel.angular.z == 0.0:
                    state = 1

            # if not ball_ok or not robot_ok:
            #     msg.cmd_vel.linear.x = 0.0 * np.cos(angle - robot_ang)
            #     msg.cmd_vel.linear.y = 0.0 * np.sin(angle - robot_ang)
            #     msg.cmd_vel.angular.z == 0.0



        elif state == 1:
            print("state ", state)
            msg.dribbler = True
            # print(np.random.rand(-3, 3))
            current_angle = robot_ang

            state = 2

        elif state == 2:

            print("state ", robot_ang)
            
            if current_angle > 0:
                msg.cmd_vel.angular.z = - 0.7
            else:
                msg.cmd_vel.angular.z = 0.7


            if (np.abs(current_angle - robot_ang) > 3*np.random.rand()):
                msg.cmd_vel.angular.z = 0.0
                state = 3

        elif state == 3:
            print("state ", state)
            msg.dribbler = False
            msg.kicker = True
            state = 0
                


        ball_ok = False
        robot_ok = False
        publisher.publish(msg)

        """if (len(laser.ranges) > 0):
            if state == 0:
                if (min(laser.ranges) < 0.5):
                    state = 1
                else:
                    msg.linear.x  = 0.25
                    msg.angular.z  = 0.0

            elif state == 1:
                state = 2
                #print(np.random)
                if (np.random.rand() < 0.5):
                    msg.angular.z  = 0.25
                    msg.linear.x  = 0.0
                else:
                    msg.angular.z  = -0.25
                    msg.linear.x  = 0.0

            elif state == 2:
                if (min(laser.ranges) > 0.5):
                    state = 0
                    
            


        else:
            msg.linear.x  = 0.0
            msg.angular.z  = 0.0 


        publisher.publish(msg)
                    """

        rate.sleep()