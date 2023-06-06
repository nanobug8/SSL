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


<<<<<<< Updated upstream
class Golero(Player):
    def shortcut(self):
        print("???")

class Referi(Player):

    golesAmarillo = 0
    golesAzul = 0
    def tanteador(self):
        print(self.golesAmarillo," - ",self.golesAzul)


    def is_yellow_goal(x1_y, y1_y, x2_y, y2_y, x3_y, y3_y, x4_y, y4_y):
        if min(x1_y, x2_y, x3_y, x4_y) <= vision.balls[0].x <= max(x1_y, x2_y, x3_y, x4_y) and \
                min(y1_y, y2_y, y3_y, y4_y) <= vision.balls[0].y<= max(y1_y, y2_y, y3_y, y4_y):
            print("Goooooooooooooooooooooool Amarillo")


    def is_blue_goal(x1_b, y1_b, x2_b, y2_b, x3_b, y3_b, x4_b, y4_b):
        if min(x1_b, x2_b, x3_b, x4_b) <= vision.balls[0].x <= max(x1_b, x2_b, x3_b, x4_b) and \
                min(y1_b, y2_b, y3_b, y4_b) <= vision.balls[0].y <= max(y1_b, y2_b, y3_b, y4_b):
            print("Goooooooooooooooooooooool Azul")


=======
>>>>>>> Stashed changes


