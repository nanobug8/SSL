#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import rospy
#from ssl_msgs.msg import SSL_DetectionFrame
from geometry_msgs.msg import Twist


vision = SSL_DetectionFrame()



def vision_callback(msg):
    # Verifica si se detectaron arcos en la visión
    if msg.balls:
        print(msg.balls)
        # Itera sobre todos los arcos detectados
        for ball in vision.balls:
            # Calcula el ángulo necesario para alinearse con el arco
            angle_to_arc = -ball.pos.x  # Suponiendo que la posición x representa el desplazamiento lateral del arco

            # Crea un mensaje Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.angular.z = angle_to_arc  # Ajusta la velocidad angular para alinearse con el arco

            # Publica el mensaje Twist en el tópico de movimiento del robot
            #pub.publish(twist_msg)

def main():
    # Inicializa el nodo de ROS
    rospy.init_node('alineacion_arco_node', anonymous=False)

    # Suscríbete al tópico de visión que proporciona los datos de detección de arcos
    rospy.Subscriber('/vision', SSL_DetectionFrame, vision_callback)

    # Crea un publicador para enviar comandos de movimiento al robot
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        if (len(vision.balls) > 0):
            ball_x = vision.balls[0].x
            ball_y = vision.balls[0].y
            print("Ball ", ball_x, ball_y)
            print("\n")

    # Mantén el nodo activo
    rospy.spin()

if __name__ == '__main__':
    main()
