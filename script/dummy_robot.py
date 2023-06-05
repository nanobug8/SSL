#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math



vision = SSL_DetectionFrame()

def vision_callback(data):
    global vision
    vision = data



def is_yellow_goal(x1_y, y1_y, x2_y, y2_y, x3_y, y3_y, x4_y, y4_y):
    if min(x1_y, x2_y, x3_y, x4_y) <= vision.balls[0].x <= max(x1_y, x2_y, x3_y, x4_y) and \
            min(y1_y, y2_y, y3_y, y4_y) <= vision.balls[0].y<= max(y1_y, y2_y, y3_y, y4_y):
        print("Goooooooooooooooooooooool Amarillo")
        #pelota al medio, un reset.



def is_blue_goal(x1_b, y1_b, x2_b, y2_b, x3_b, y3_b, x4_b, y4_b):
    if min(x1_b, x2_b, x3_b, x4_b) <= vision.balls[0].x <= max(x1_b, x2_b, x3_b, x4_b) and \
            min(y1_b, y2_b, y3_b, y4_b) <= vision.balls[0].y <= max(y1_b, y2_b, y3_b, y4_b):
        print("Goooooooooooooooooooooool Azul")
        #pelota al medio, un reset.

def find_arc(message):
    print(message)
    if message.goals:
        # Itera sobre todos los arcos detectados
        for goal in message.goals:
            # Obtiene la posición del arco
            x = goal.x
            y = goal.y

            # Imprime la posición del arco
            print("Posición del arco:")
            print("Coordenadas (x, y):", x, y)

def obtener_orientacion(x, y):
    # Calcular el ángulo de orientación utilizando la función atan2
    orientacion = math.atan2(y, x)

    # Convertir el ángulo de radianes a grados
    #orientacion_grados = math.degrees(orientacion)

    return orientacion

def obtener_orientacion_2(actual_x, actual_y, destino_x, destino_y):
    # Calcular el ángulo de orientación utilizando la función atan2
    diff_x = destino_x - actual_x
    diff_y = destino_y - actual_y

    # Calcular el ángulo de orientación utilizando la función atan2
    #orientacion = math.atan2(diff_y, diff_x)
    direction = np.arctan2((actual_y - destino_y), (actual_x - destino_x))
    #direction_grados = math.degrees(direction)

    return direction

def calcular_angulo_giro(orientacion_actual, orientacion_arco):
    # Calcular la diferencia angular entre la orientación actual y la orientación deseada
    angulo_giro = orientacion_arco - orientacion_actual

    # Asegurarse de que el ángulo de giro esté en el rango de -180 a 180 grados
    angulo_giro = (angulo_giro + 180) % 360 - 180

    return angulo_giro


def ir_a_punto(actual_x, actual_y, punto_x, punto_y):


    # Calcular la diferencia entre la posición actual y el punto objetivo
    diferencia_x = punto_x - actual_x
    diferencia_y = punto_y - actual_y

    # Calcular la distancia y el ángulo hacia el punto objetivo
    #distancia = ((diferencia_x ** 2) + (diferencia_y ** 2)) ** 0.5
    angle = np.arctan2(diferencia_x, diferencia_y)
    #angulo = math.atan2(diferencia_y, diferencia_x)

    # Definir la velocidad lineal y angular
    return angle




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

            #print("state ", state)
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
            #print("state ", state)
            msg.dribbler = True
            # print(np.random.rand(-3, 3))
            current_angle = robot_ang

            state = 2

        elif state == 2:

            #print("state ", robot_ang)

            #buscar arco.
            angulo = ir_a_punto(vision.robots_blue[0].x,vision.robots_blue[0].y,4500,0)
            '''
            giro = calcular_angulo_giro(vision.robots_blue[0].orientation,orientacion_arco)
            print(giro)
            print(robot_ang)
            '''
            print(robot_ang)
            print(angulo)
            if (angulo - robot_ang > angle_tol):
                msg.cmd_vel.angular.z = 0.7
            elif (angulo - robot_ang < - angle_tol):
                msg.cmd_vel.angular.z = - 0.7
            else:
                msg.cmd_vel.angular.z = 0.0

            if dist > 120:
                msg.cmd_vel.linear.x = 0.3 * np.cos(angle - robot_ang)
                msg.cmd_vel.linear.y = 0.3 * np.sin(angle - robot_ang)
            else:
                msg.cmd_vel.linear.x = 0.0 * np.cos(angle - robot_ang)
                msg.cmd_vel.linear.y = 0.0 * np.sin(angle - robot_ang)
                if msg.cmd_vel.angular.z == 0.0:
                    state = 3




            '''
            if (np.abs(current_angle - robot_ang) > 3*np.random.rand()):
                msg.cmd_vel.angular.z = 0.0
            '''



        elif state == 3:
            #print("state ", state)
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

        is_yellow_goal(-4500,1000,-3500,1000,-4500,3000,-3500,3000)
