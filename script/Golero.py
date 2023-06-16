#! /usr/bin/env python3
import math
import random
import rospy
import sys
import numpy as np
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *


vision = SSL_DetectionFrame()

class Player:
    def __init__(self,team,id):
        self.topic = "/robot_{}_{}/cmd".format(team, id)
        self.pub = rospy.Publisher(self.topic, SSL, queue_size = 10)
        self.vision_sub = rospy.Subscriber("/vision", SSL_DetectionFrame, self.vision_callback)
        self.id = id
        self.x = 0
        self.y = 0
        self.ball_x = 0
        self.ball_y = 0
        self.ball_angle = 0
        self.ball_dist = [1e4,1e4,1e4,1e4,1e4]
        self.goal_x = 2050
        self.goal_y = 0
        self.goal_x_blue = -2050
        self.goal_y_blue = 0
        self.goal_angle = 0
        self.goal_dist = [0,0,0,0,0]
        self.goal_angle_blue = 0
        self.goal_dist_blue = 0
        self.mate_x = [0,0,0,0,0]
        self.mate_y = [0,0,0,0,0]
        self.mate_yaw = [0,0,0,0,0]
        self.mate_angle = [0,0,0,0,0]
        self.mate_dist = [0,0,0,0,0]
        #self.arc_dist=[0,0,0,0,0]
        self.closest_mate = self.id
        self.closest_to_ball = self.id
        self.closet_to_arc = self.id
        self.yaw = 0
        self.angle_tolerance = 0.06
        self.msg = SSL()
        self.state = 0
        self.team = team
        self.start_time = rospy.get_time()
        self.side = 0
        self.tackle_angle = 0
        self.position = False


    def vision_callback(self,data):

        if (len(data.balls) > 0):
            self.ball_x = data.balls[0].x
            self.ball_y = data.balls[0].y
            # print("Ball ", vision.balls[0].x, vision.balls[0].y, "\n")

        if self.team == "yellow":

            robots = data.robots_yellow
        else:
            robots = data.robots_blue

        for robot in robots:
            self.mate_x[robot.robot_id] = robot.x
            self.mate_y[robot.robot_id] = robot.y
            self.mate_yaw[robot.robot_id] = robot.orientation
            self.ball_dist[robot.robot_id] = np.sqrt(np.abs(self.ball_x - robot.x)**2 + np.abs(self.ball_y - robot.y)**2)
            self.goal_dist[robot.robot_id] = np.sqrt(np.abs(self.goal_x - robot.x)**2 + np.abs(self.goal_y - robot.y)**2)
            self.mate_dist[robot.robot_id] = np.sqrt(np.abs(self.x - robot.x)**2 + np.abs(self.y - robot.y)**2)
            self.mate_angle[robot.robot_id] = np.arctan2((robot.y - self.y), (robot.x - self.x)) if robot.robot_id != self.id else 0
            #self.arc_dist[robot.robot_id] = np.sqrt(np.abs( self.goal_x - robot.x)**2 + np.abs(self.goal_y - robot.y)**2)

        self.mate_dist[self.id] = 1e7

        #Save my own data
        self.x = self.mate_x[self.id]
        self.y = self.mate_y[self.id]
        self.yaw = self.mate_yaw[self.id]

        self.closest_mate = self.mate_dist.index(min(self.mate_dist))
        self.closest_to_ball = self.ball_dist.index(min(self.ball_dist))
        self.closet_to_arc = self.goal_dist.index(min(self.goal_dist))

        if np.abs(self.ball_y - self.y) > 0:
            self.ball_angle = np.arctan2((self.ball_y - self.y), (self.ball_x - self.x))

        if np.abs(self.goal_y - self.y) > 0:
            self.goal_angle = np.arctan2((self.goal_y - self.y), (self.goal_x - self.x))

        if np.abs(self.goal_y_blue - self.y) > 0:
            self.goal_angle_blue = np.arctan2((self.goal_y_blue - self.y), (self.goal_x_blue - self.x))
        
        self.goal_dist_blue = np.sqrt(np.abs(self.goal_x_blue - self.x)**2 + np.abs(self.goal_y_blue - self.y)**2)

        print("estado golero           ", self.goal_dist_blue)

        if self.state == 0:
           self.catch_the_ball()

        elif self.state == 1:
            self.intercept()

        elif self.state == 2:
            self.get_the_ball()

        elif self.state == 3:
            self.drive_to_goal()

        
    def drive_to_goal(self):
        if np.abs(self.goal_angle_blue - self.yaw) < 0.07 :
            self.msg.cmd_vel.angular.z = self.goal_angle_blue - self.yaw
        else:
            aim = True
            self.msg.cmd_vel.angular.z = 0.0

        self.speed_scale = 2

        self.msg.cmd_vel.linear.x = self.speed_scale * np.cos(self.goal_angle_blue - self.yaw)
        self.msg.cmd_vel.linear.y = self.speed_scale * np.sin(self.goal_angle_blue - self.yaw)

        self.msg.dribbler = True
        self.msg.kicker = False

        if self.goal_dist_blue < 2800:
            self.msg.dribbler = False
            self.msg.kicker = True
            self.state = 0

        self.pub.publish(self.msg)

    def get_random_point(self,x_min, x_max, y_min, y_max):
        random_x = random.uniform(x_min, x_max)
        random_y = random.uniform(y_min, y_max)
        return random_x, random_y

    def intercept(self):

        if self.goal_dist[self.id] > 1000:
            self.state = 0

        elif self.ball_dist[self.id] < 700:

            self.msg.dribbler = True
            self.msg.kicker = False

            self.state = 2

        #if self.ball_dist[self.id] <= 1000:

            #x = (self.x + random_posX) / 2 + amplitude * (random_posX - self.x) * (1 + math.sin(2 * math.pi / period * current_time)) / 2
            #y = (self.y + random_posY) / 2 + amplitude * (random_posY - self.y) * (1 + math.sin(2 * math.pi / period * current_time)) / 2
            #self.msg.cmd_vel.linear.x = 0
            #self.msg.cmd_vel.linear.x = 0.3 * np.cos(angular_intercept - self.yaw)
            #self.msg.cmd_vel.linear.y = 0.3 * np.sin(angular_intercept - self.yaw)
            #self.msg.cmd_vel.angular.z = 0
            #self.msg.cmd_vel.angular.z = angular_intercept - self.yaw
            #print(x,y)



            #self.state = 4

        else:
            self.state = 0

    def stop(self):
        self.pub.publish()

    def catch_the_ball(self):

        if (np.abs(self.goal_angle - self.yaw) > self.angle_tolerance):

            self.msg.cmd_vel.angular.z = self.goal_angle - self.yaw
        else:
            self.msg.cmd_vel.angular.z = 0.0

        if self.goal_dist[self.id] > 120:
            self.msg.cmd_vel.linear.x = 0.3 * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.3 * np.sin(self.goal_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.goal_angle - self.yaw)
            self.msg.dribbler = True
            self.msg.kicker = False

        if min(2000, 2000, 1800, 1800) <= self.x <= max(2000, 2000, 1800, 1800) and \
           min(400, -400, 400, -400) <= self.y <= max(400, -400, 400, -400):
            self.state = 1

        if (np.abs(self.ball_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.yaw - self.ball_angle
        else:
            self.msg.cmd_vel.angular.z = 0.0

        self.msg.dribbler = True
        self.msg.kicker = False

        self.pub.publish(self.msg)


    def get_the_ball(self):

        if (np.abs(self.ball_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.ball_angle - self.yaw
        else:
            self.msg.cmd_vel.angular.z = 0.0

        if self.goal_dist[self.id] > 1000:
            self.state = 0
        elif self.ball_dist[self.id] > 120:
            self.msg.cmd_vel.linear.x = 0.3 * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.3 * np.sin(self.ball_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.ball_angle - self.yaw)
            self.msg.dribbler = True
            self.msg.kicker = False
            print('Finish Demo')
            self.state = 3
            # if msg.cmd_vel.angular.z == 0.0:
            #     state = 1

        self.pub.publish(self.msg)



if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("No enough arguments. Exiting...")
    else:
        node_name = "SSL_{}_{}".format(sys.argv[1],sys.argv[2])
        rospy.init_node(node_name, anonymous = True)

        player = Player(sys.argv[1], int(sys.argv[2]))
        rospy.spin()

        #rate = rospy.Rate(10)
        #while not rospy.is_shutdown():
        #rate.sleep(10)







