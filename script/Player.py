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
<<<<<<< HEAD

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




=======
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
>>>>>>> origin/master
