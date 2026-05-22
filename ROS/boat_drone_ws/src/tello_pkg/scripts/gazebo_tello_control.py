#!/usr/bin/env python3
"""
Script utilizado para controlar o Tello na simulação.
"""
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class tello_control:

    #velocidade do drone 0 a 1.0
    speed = 0.50

    #teclas de comando de velocidade
    move_keys = {
        'w':(speed,0,0,0),      #Mover para frente
        'a':(0,speed,0,0),      #Mover para esquerda
        's':(-speed,0,0,0),     #Mover para trás
        'd':(0,-speed,0,0),     #Mover para direita
        '0':(0,0,0,0),          #Ficar parado
        '4':(0,0,0,0.7*speed),    #Girar sentido anti-horário
        '6':(0,0,0,-0.7*speed),   #Girar sentido horário
        '8':(0,0,speed,0),      #Mover para cima
        '5':(0,0,-speed,0),     #Mover para baixo
    }

    #teclas de funções
    takeoff_land_keys = {
        'l':'l',                #Pousar
        't':'t',                #Decolar
        '\x03':'\x03',          #Sair
    }

    #mensagem inicial
    txt_init = """
                   PRESSIONE PARA MOVER:
                  (desabilite o capslock)
                   
    w: frente           8: cima
    s: trás             5: baixo
    d: direita          6: girar sentido horário
    a: esquerda         4: girar sentido anti-horário

    t: decolar          l: pousar

    0: ficar parado     
    """

    #mensagem de erro quando pressionar tecla errada
    txt_erro = """
    ___________________________________________________
    
                   BOTÃO INVÁLIDO!!! 
    ___________________________________________________
    """

    def __init__(self):
        # Iniciar ROS node
        rospy.init_node('tello_control',anonymous=False)

        # Publishers (os comentados não são funções básicas)
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        #self.pub_throw_takeoff = rospy.Publisher('throw_takeoff', Empty,  queue_size=1, latch=False)
        #self.pub_palm_land = rospy.Publisher('palm_land', Empty,  queue_size=1, latch=False)
        #self.pub_flip = rospy.Publisher('flip', UInt8,  queue_size=1, latch=False)
        #self.pub_fast_mode = rospy.Publisher('fast_mode', Bool,  queue_size=1, latch=False)
        
        rospy.loginfo('Control node initialized')
        print(self.txt_init)

    #função para enviar comandos de velocidade para o drone
    def set_cmd_vel(self,vlx,vly,vlz,vaz):
        vel_msg = Twist()
        vel_msg.linear.x = float(vlx)
        vel_msg.linear.y = float(vly)
        vel_msg.linear.z = float(vlz)
        vel_msg.angular.z = float(vaz)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        self.pub_cmd_vel.publish(vel_msg)

    #função para ler qual tecla foi apertada baseado no teleop_twist_keyboard
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    #função para realizar ações de acordo com a tecla apertada
    def move(self,key):
        if key in self.move_keys.keys():
            vlx = self.move_keys[key][0]
            vly = self.move_keys[key][1]
            vlz = self.move_keys[key][2]
            vaz = self.move_keys[key][3]

            self.set_cmd_vel(vlx,vly,vlz,vaz)
        elif key in self.takeoff_land_keys.keys():
            if (key == 't'): self.takeoff()
            if (key == 'l'): self.land()
        else:
            vlx = 0
            vly = 0
            vlz = 0
            vaz = 0
            print(self.txt_erro)
            print(self.txt_init)

    def land(self):
        self.pub_land.publish()
    
    def takeoff(self):
        self.pub_takeoff.publish()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        drone = tello_control()

        while True:
            key = drone.getKey()
            drone.move(key)
            if(key == '\x03'):
                break

    except rospy.ROSInterruptException:
        pass

    finally:
        drone.land()
