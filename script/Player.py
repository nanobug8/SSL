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

    orientacion_arco = 0
    def _init_(self, nombre):
        self.nombre = nombre

    def defend(self):
        print("Posicionarse frente a la pelota, e ir hacia ella")

    def soccer_pass(self):
        print("Hacer un pase al compa")

    def positioning(self):
        print("Posicionarse")

    def drive_to_goal(self,robot_x,robot_y,robot_ang):
        print("Ir hacia el arco")
        orientacion_arco=obtener_orientacion(robot_x,robot_y,2000,0)
        giro = calcular_angulo_giro(robot_ang,orientacion_arco)


        return giro


    def shoot(self):
        #llamado a findgoal
        print("Tirar al arco")


def obtener_orientacion(actual_x, actual_y, destino_x, destino_y):
    # Calcular la diferencia en coordenadas x e y
    diff_x = destino_x - actual_x
    diff_y = destino_y - actual_y

    # Calcular el ángulo de orientación utilizando la función atan2
    orientacion = np.arctan2(diff_y, diff_x)

    # Converti el ángulo de radianes a grados
    #orientacion_grados = math.degrees(orientacion)

    print('orientacion', orientacion)

    return orientacion

def calcular_angulo_giro(orientacion_actual, orientacion_arco):
    # Calcular la diferencia angular entre la orientación actual y la orientación deseada
    angulo_giro = orientacion_arco - orientacion_actual

    # Asegurarse de que el ángulo de giro esté en el rango de -180 a 180 grados
    #angulo_giro = (angulo_giro + 180) % 360 - 180

    print('angulo_giro', angulo_giro)
    return angulo_giro


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




