#! /usr/bin/env python3
import math

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *


vision = SSL_DetectionFrame()

class Player:
    def __init__(self,id):
        self.pub = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size = 10)
        self.vision_sub = rospy.Subscriber("/vision", SSL_DetectionFrame, self.vision_callback)
        self.id = id
        self.x = 0
        self.y = 0
        self.ball_x = 0
        self.ball_y = 0
        self.ball_angle = 0
        self.ball_dist = 0
        self.goal_x = 2000
        self.goal_y = 0
        self.goal_angle = 0
        self.goal_dist = 0
        self.mate_x = 0
        self.mate_y = 0
        self.mate_angle = 0
        self.mate_dist = 0
        self.yaw = 0
        self.angle_tolerance = 0.06
        self.msg = SSL()
        self.state = 0

    def vision_callback(self,data):

        if (len(data.balls) > 0):
            self.ball_x = data.balls[0].x
            self.ball_y = data.balls[0].y
            # print("Ball ", vision.balls[0].x, vision.balls[0].y, "\n")

        for robot in data.robots_blue:
            if robot.robot_id == self.id:
                self.x = robot.x
                self.y = robot.y
                self.yaw = robot.orientation

        if np.abs(self.ball_y - self.y) > 0:
            self.ball_angle = np.arctan2((self.ball_y - self.y), (self.ball_x - self.x))
            self.ball_dist = np.sqrt(np.abs(self.ball_x - self.x)**2 + np.abs(self.ball_y - self.y)**2) 

        if np.abs(self.goal_y - self.y) > 0:
            self.goal_angle = np.arctan2((self.goal_y - self.y), (self.goal_x - self.x))
            self.goal_dist = np.sqrt(np.abs(self.goal_x - self.x)**2 + np.abs(self.goal_y - self.y)**2)

        if self.state == 0:
            self.get_the_ball()
        elif self.state == 1:
            self.drive_to_goal()


    def get_the_ball(self):

        if (np.abs(self.ball_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.ball_angle - self.yaw
        else:
            self.msg.cmd_vel.angular.z = 0.0

        if self.ball_dist > 120:
            self.msg.cmd_vel.linear.x = 0.3 * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.3 * np.sin(self.ball_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.ball_angle - self.yaw)
            self.msg.dribbler = True
            self.msg.kicker = False
            self.state = 1
            # if msg.cmd_vel.angular.z == 0.0:
            #     state = 1
        
        self.pub.publish(self.msg)

    
    def drive_to_goal(self):

        if (np.abs(self.goal_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.goal_angle - self.yaw
        else:
            self.msg.cmd_vel.angular.z = 0.0

        if self.goal_dist > 120:
            self.msg.cmd_vel.linear.x = 0.3 * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.3 * np.sin(self.goal_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.goal_angle - self.yaw)
            self.msg.dribbler = False
            self.msg.kicker = True

        self.pub.publish(self.msg)





if __name__ == '__main__':

    rospy.init_node("stage_controller_node", anonymous = False)
    player = Player(0)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()