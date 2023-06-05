vision = SSL_DetectionFrame()

class Player:
    def __init__(self, nombre):
        self.nombre = nombre

class Defensa(Player):

    def defend(self):
        print("Posicionarse frente a la pelota, e ir hacia ella")
    def soccer_pass(self):
        print("Hacer un pase al compa")


class Delantero(Player):
    def find_goal(self):
        print("Encontrar la posicion del arco")

        '''
        goal_angle = math.atan2(ball_y - robot0_y, ball_x - robot0_x)
		
		heading = abs(goal_angle - robot0.orientation)
		
		distance = math.sqrt((ball_x - robot0_x)**2 + (ball_y - robot0_y)**2)
		
		if (distance < 0.2):
			msg.cmd_vel.linear.x = 0
			msg.cmd_vel.angular.z = 0
		else:
			if (heading < 0.2):
				msg.cmd_vel.linear.x = 0.25
				msg.cmd_vel.angular.z = 0
			else:
				msg.cmd_vel.linear.x = 0.
				msg.cmd_vel.angular.z = 0.5
        '''

    def shoot(self):
        #llamado a findgoal
        print("Tirar al arco")


class Golero(Player):
    def shortcut(self):
        print("???")


def is_yellow_goal(x1_y, y1_y, x2_y, y2_y, x3_y, y3_y, x4_y, y4_y):
    if min(x1_y, x2_y, x3_y, x4_y) <= vision.balls[0].x <= max(x1_y, x2_y, x3_y, x4_y) and \
            min(y1_y, y2_y, y3_y, y4_y) <= vision.balls[0].y<= max(y1_y, y2_y, y3_y, y4_y):
        print("Goooooooooooooooooooooool Amarillo")


def is_blue_goal(x1_b, y1_b, x2_b, y2_b, x3_b, y3_b, x4_b, y4_b):
    if min(x1_b, x2_b, x3_b, x4_b) <= vision.balls[0].x <= max(x1_b, x2_b, x3_b, x4_b) and \
            min(y1_b, y2_b, y3_b, y4_b) <= vision.balls[0].y <= max(y1_b, y2_b, y3_b, y4_b):
        print("Goooooooooooooooooooooool Azul")




