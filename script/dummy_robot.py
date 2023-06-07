#! /usr/bin/env python3
import math

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
from Player import *


vision = SSL_DetectionFrame()

ball_x = 0
ball_y = 0
robot_x = 0
robot_y = 0
robot_ang = 0

def vision_callback(data):
    global vision
    global ball_x
    global ball_y
    global robot_x
    global robot_y
    global robot_ang

    vision = data

    if (len(vision.balls) > 0):
        ball_x = vision.balls[0].x
        ball_y = vision.balls[0].y
        # print("Ball ", vision.balls[0].x, vision.balls[0].y, "\n")

    for robot in vision.robots_blue:
        if robot.robot_id == 0:
            robot_x = robot.x
            robot_y = robot.y
            robot_ang = robot.orientation
                # print("Robot ", robot_x, robot_y, robot_ang, "\n")
    # print(data.camera_id)


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

    angle_tol = 0.06

    state = 0

    playerBlue0 = Player()

    while not rospy.is_shutdown():

        if np.abs(ball_y - robot_y) > 0:
            angle = np.arctan2((ball_y - robot_y), (ball_x - robot_x))
            dist = np.sqrt(np.abs(ball_x - robot_x)**2 + np.abs(ball_y - robot_y)**2)

        # print(robot_ang)
        # print(angle)
        #print(angle, robot_ang, angle - robot_ang)

        if state == 0:

            # print("state ", state)
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
            # print("state ", state)
            msg.dribbler = True
            # print(np.random.rand(-3, 3))
            current_angle = robot_ang

            state = 2

        elif state == 2:

            # print("state ", robot_ang)
            orientacion_arco=obtener_orientacion(robot_x,robot_y,2000,0)
            giro = calcular_angulo_giro(robot_ang,orientacion_arco)

            #playerBlue0.drive_to_goal(robot_x,robot_y,robot_ang)

            msg.cmd_vel.angular.z = giro

            if abs(robot_ang - giro) < angle_tol:
                state = 3
                print(state)
            '''
            if np.abs(current_angle - giro) > angle_tol:
                msg.cmd_vel.angular.z = giro + giro/math.pi
            else:
                msg.cmd_vel.angular.z =  giro/math.pi - giro

            if (np.abs(current_angle - robot_ang) > 3*np.random.rand()):
                msg.cmd_vel.angular.z = 0.0
                state = 3
            '''



        elif state == 3:
            # print("state ", state)
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