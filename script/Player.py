#! /usr/bin/env python3
import math

import rospy
import sys
import numpy as np
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
        self.team = team
        self.x = 0
        self.y = 0
        self.ball_x = 0
        self.ball_y = 0
        self.ball_angle = 0
        self.ball_dist = [1e4,1e4,1e4,1e4,1e4]
        self.goal_x = 2000
        self.goal_y = 0
        self.goal_angle = 0
        self.goal_dist = [1e4,1e4,1e4,1e4,1e4]
        self.mate_x = [0,0,0,0,0]
        self.mate_y = [0,0,0,0,0]
        self.mate_yaw = [0,0,0,0,0]
        self.mate_angle = [0,0,0,0,0]
        self.mate_dist = [1e4,1e4,1e4,1e4,1e4]
        self.closest_mate = self.id
        self.closest_to_ball = self.id
        self.yaw = 0
        self.angle_tolerance = 0.12
        self.msg = SSL()
        self.state = 0
        self.speed_scale = 0.3
        self.ang_scale = 3
        self.angle_right_post = 0
        self.angle_left_post = 0


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
        
        self.mate_dist[self.id] = 1e7

        #Save my own data
        self.x = self.mate_x[self.id]
        self.y = self.mate_y[self.id]
        self.yaw = self.mate_yaw[self.id]

        #Who are the closest?
        self.closest_mate = self.mate_dist.index(min(self.mate_dist))
        self.closest_to_ball = self.ball_dist.index(min(self.ball_dist))
        self.closest_to_goal = self.goal_dist.index(min(self.goal_dist))

        #Angle range to goal
        self.angle_right_post = np.arctan2((self.goal_y - 500 - self.y), (self.goal_x - self.x))
        self.angle_left_post = np.arctan2((self.goal_y + 500 - self.y), (self.goal_x - self.x))

        if np.abs(self.ball_y - self.y) > 0:
            self.ball_angle = np.arctan2((self.ball_y - self.y), (self.ball_x - self.x))
            #self.ball_dist = np.sqrt(np.abs(self.ball_x - self.x)**2 + np.abs(self.ball_y - self.y)**2) 

        if np.abs(self.goal_y - self.y) > 0:
            self.goal_angle = np.arctan2((self.goal_y - self.y), (self.goal_x - self.x))
            #self.goal_dist = np.sqrt(np.abs(self.goal_x - self.x)**2 + np.abs(self.goal_y - self.y)**2)


        #Comportamental control
        if self.closest_to_ball == self.id:
            
            if self.state == 0:
                print(self.id, " yendo \n")
                self.speed_scale = 0.8
                self.get_the_ball()
            elif self.state == 1:
                if self.ball_dist[self.id] > 300:
                    print(self.id, " trancado \n")
                    self.state = 0
                    self.stop()
                else:
                    self.speed_scale = 0.5
                    if self.goal_dist[self.id] - self.goal_dist[self.closest_to_goal] > 100 and self.closest_to_goal != self.id:
                        print(self.id, " pasando \n")
                        self.pass_to_mate()
                    else:
                        print(self.id, " chutando \n")
                        self.drive_to_goal()
                        if self.ball_dist[self.id] > 120:
                            self.state = 0
            # self.stop()
        else:
            self.speed_scale = 0.7
            self.state = 0
            self.move_without_ball()
            
        if self.goal():
            self.stop()

        # if self.state == 0:
        #     self.get_the_ball()
        # elif self.state == 1:
        #     self.drive_to_goal()
        #     # self.pass_to_mate()
        # elif self.state == 2:
        #     self.move_without_ball()

    def stop(self):
        self.pub.publish()

    def goal(self):
        if self.ball_y < 500 and self.ball_y > -500 and self.ball_x > 2000:
            return True
        else:
            return False

    def get_the_ball(self):

        aim = False
        if (np.abs(self.ball_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.ang_scale * (self.ball_angle - self.yaw)
        else:
            aim = True
            self.msg.cmd_vel.angular.z = 0.0

        if self.ball_dist[self.id] > 120:
            self.msg.cmd_vel.linear.x = self.speed_scale * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = self.speed_scale * np.sin(self.ball_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.ball_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.ball_angle - self.yaw)
            self.msg.dribbler = aim
            self.msg.kicker = False
            if aim:
                self.state = 1
            # if msg.cmd_vel.angular.z == 0.0:
            #     state = 1
        
        self.pub.publish(self.msg)

    
    def drive_to_goal(self):
        aim = False
        if np.abs(self.goal_angle - self.yaw) < self.angle_right_post or np.abs(self.goal_angle - self.yaw) > self.angle_left_post :
            self.msg.cmd_vel.angular.z = self.goal_angle - self.yaw
        else:
            aim = True
            self.msg.cmd_vel.angular.z = 0.0

        if self.goal_dist[self.id] > 600:
            self.msg.cmd_vel.linear.x = self.speed_scale * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = self.speed_scale * np.sin(self.goal_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.goal_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.goal_angle - self.yaw)
            self.msg.dribbler = not aim
            self.msg.kicker = aim

        self.pub.publish(self.msg)


    def pass_to_mate(self):

        aim = False
        if (np.abs(self.mate_angle[self.closest_mate] - self.yaw) > self.angle_tolerance):
            # self.msg.cmd_vel.angular.z = self.mate_angle[self.closest_mate] - self.yaw
            self.msg.cmd_vel.angular.z = self.ang_scale * (self.mate_angle[self.closest_to_goal] - self.yaw)
        else:
            aim = True
            self.msg.cmd_vel.angular.z = 0.0

        if self.mate_dist[self.closest_mate] > 1500:
            self.msg.cmd_vel.linear.x = self.speed_scale * np.cos(self.mate_angle[self.closest_to_goal] - self.yaw)
            self.msg.cmd_vel.linear.y = self.speed_scale * np.sin(self.mate_angle[self.closest_to_goal] - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(self.mate_angle[self.closest_to_goal] - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(self.mate_angle[self.closest_to_goal] - self.yaw)
            self.msg.dribbler = not aim
            self.msg.kicker = aim

        self.pub.publish(self.msg)

    def move_without_ball(self):

        next_x = (min(700, (2000 - self.ball_x)) + self.ball_x) / 2
        next_y = ((-1 * np.sign(self.ball_y))*1500 + self.ball_y) / 2
        next_angle = np.arctan2((next_y - self.y), (next_x - self.x))
        next_dist = np.sqrt(np.abs(self.x - next_x)**2 + np.abs(self.y - next_y)**2)

        if next_dist > 150:
            self.msg.cmd_vel.linear.x = self.speed_scale * np.cos(next_angle - self.yaw)
            self.msg.cmd_vel.linear.y = self.speed_scale * np.sin(next_angle - self.yaw)
        else:
            self.msg.cmd_vel.linear.x = 0.0 * np.cos(next_angle - self.yaw)
            self.msg.cmd_vel.linear.y = 0.0 * np.sin(next_angle - self.yaw)
            
        if (np.abs(self.ball_angle - self.yaw) > self.angle_tolerance):
            self.msg.cmd_vel.angular.z = self.ball_angle - self.yaw
            # self.msg.cmd_vel.angular.z *= 10
        else:
            self.msg.cmd_vel.angular.z = 0.0

        self.pub.publish(self.msg)



if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("No enough arguments. Exiting...")
    else:
        node_name = "SSL_{}_{}".format(sys.argv[1],sys.argv[2])
        rospy.init_node(node_name, anonymous = True)

        player = Player(sys.argv[1], int(sys.argv[2]))
        rospy.spin()

    # rate = rospy.Rate(10)    
    # while not rospy.is_shutdown():
    #     rate.sleep()