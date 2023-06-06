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
    def __init__(self, nombre):
        self.nombre = nombre
        
    def defend(self):
        print("Posicionarse frente a la pelota, e ir hacia ella")

    def soccer_pass(self):
        print("Hacer un pase al compa")

    def positioning(self):
        print("Posicionarse")

    def drive_to_goal(self):
        print("Ir hacia el arco")

    def shoot(self):
        #llamado a findgoal
        print("Tirar al arco")




