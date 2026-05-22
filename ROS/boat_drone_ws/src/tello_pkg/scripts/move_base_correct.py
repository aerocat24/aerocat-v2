#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def twist_callback(msg):
    # Criar nova mensagem Twist
    new_msg = Twist()

    # Inverter os valores de linear.x e linear.y
    new_msg.linear.x = msg.linear.y
    new_msg.linear.y = msg.linear.x
    new_msg.angular.z = -msg.angular.z
    # Manter os outros valores intactos
    new_msg.linear.z = msg.linear.z
    new_msg.angular.x = msg.angular.x
    new_msg.angular.y = msg.angular.y


    # Publicar no novo tópico
    pub.publish(new_msg)

if __name__ == '__main__':
    # Inicializa o nó
    rospy.init_node('twist_inverter', anonymous=True)

    # Criar o publicador no tópico /cmd_vel_output
    pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)

    # Inscrever-se no tópico /cmd_vel_input
    rospy.Subscriber('/cmd_vel_move_base', Twist, twist_callback)

    # Manter o nó rodando
    rospy.spin()
