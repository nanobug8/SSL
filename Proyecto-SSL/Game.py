#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from grsim_ros_bridge_msgs.msg import SSL
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionRobot, SSL_DetectionBall
import math
from Player import Golero, Defensa, Delantero


ball = SSL_DetectionBall()
robot0 = SSL_DetectionRobot()
robot1 = SSL_DetectionRobot()
robot2 = SSL_DetectionRobot()
robot3 = SSL_DetectionRobot()
robot4 = SSL_DetectionRobot()

if __name__=="__main__":
    rospy.init_node("grsim_pria", anonymous=False)
    rospy.Subscriber("/vision", SSL_DetectionFrame, vision_callback)
    pub_robot_0_blue = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)

    r = rospy.Rate(1000)

    robot0_x = 0
    robot0_y = 0
    ball_x = 0
    ball_y = 0

    msg = SSL()

    # while grsim running
    while not rospy.is_shutdown():
        try:
            ball_x = ball[0].x
            ball_y = ball[0].y
            robot0_x = robot0.x
            robot0_y = robot0.y
        except:
            pass
    ssl_1 = Golero("Capitan")
    ssl_2 = Defensa("ZagueroDerecho")
    ssl_3 = Defensa("ZagueroIzquierdo")
    ssl_4 = Delantero("PunteroDerecho")
    ssl_5 = Delantero("PunteroIzquierdo")

ssl_2.soccer_pass()
