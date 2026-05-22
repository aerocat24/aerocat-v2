#!/usr/bin/env python3
#nova odometria do tello, 
#para posição subscresver os topicos desse nó para estimar uma posição
"""
Script utilizado para corrigir a odometria gerada pelo Tello.
"""
#imports
import rospy
from std_msgs.msg import Empty, Float32, Int32
from nav_msgs.msg import Odometry
from tello_driver.msg import TelloStatus
import tf
import math

#criação de objetos
offset_odom = Odometry()
offset_z = Float32()
last_pose = Odometry()
tf_broadcaster = tf.TransformBroadcaster()
last_x = 0
last_y = 0
last_z = 0
last_w = 1
# status possíveis = ["desligado","voando","pousando","decolando"]

status = "desligado"
#status = "decolando" # teste


# Funções:

# Função para alterar variáveis auxiliares após takeoff
def callback_takeoff(msg):
    global status
    if status == "desligado":
        rospy.sleep(4)
        status = "decolando"
    else: status = "voando"

# obtém o valor de offset do eixo Z do /tello/status
def callback_offset(msg):
    global offset_z
    offset_z = msg.height_m

# zera offset do eixo z ao pousar e ataliza variável auxiliar
def callback_land(msg):
    global status
    status = "pousando"

# recebe valores da odometria, realiza o tratamento e publica nova odometria
def callback_odom(msg, callback_args):

    pub_new_odom, pub_status_odom = callback_args  # desempacotando a tupla

    new_odom = Odometry()
    new_odom.header = msg.header
    new_odom.header.frame_id = "world"
    new_odom.child_frame_id = "base_link"
    
    global offset_odom, offset_z, status, last_pose, last_x, last_y, last_z, last_w
    
# executa a primeira vez para definir os offsets    
    if status == "decolando": 
        rospy.loginfo("offset definido")
        offset_odom = msg
        offset_odom.pose.pose.position.z     =  msg.pose.pose.position.z - offset_z
        status = "voando"
        
        status_odom = 1 # 1 = com offset, 0 = sem offset
        pub_status_odom.publish(status_odom)

# executa o processo de pouso e remove "impulso" do gráfico.
    elif status == "pousando" and offset_z <0.3:
        new_odom = msg 
        new_odom.pose.pose.position.x    =  msg.pose.pose.position.x - offset_odom.pose.pose.position.x
        new_odom.pose.pose.position.y    =  -(msg.pose.pose.position.y - offset_odom.pose.pose.position.y)
        new_odom.pose.pose.position.z    =  0

        # armazena os ultimos valores de posição x, y
        last_pose.pose.pose.position.x = new_odom.pose.pose.position.x
        last_pose.pose.pose.position.y = new_odom.pose.pose.position.y
        status = "desligado"
    
# garante que o drone está pousado e parado na ultima posição de x e y
    elif status == "desligado": 
        new_odom.pose.pose.position.x    =  last_pose.pose.pose.position.x
        new_odom.pose.pose.position.y    =  last_pose.pose.pose.position.y
        new_odom.pose.pose.position.z    =  0
        new_odom.pose.pose.orientation.w =  last_pose.pose.pose.orientation.w
        new_odom.pose.pose.orientation.x =  last_pose.pose.pose.orientation.x
        new_odom.pose.pose.orientation.y =  last_pose.pose.pose.orientation.y
        new_odom.pose.pose.orientation.z =  last_pose.pose.pose.orientation.z
        new_odom.twist.twist.linear.x    =  0
        new_odom.twist.twist.linear.y    =  0
        new_odom.twist.twist.linear.z    =  0
        new_odom.twist.twist.angular.x   =  0
        new_odom.twist.twist.angular.y   =  0
        new_odom.twist.twist.angular.z   =  0   
        #print("--------------------------------desligado--------------------------------") 

# publica a odometria durante status = voando.
    else:
        new_odom = msg
        new_odom.pose.pose.position.x    =  msg.pose.pose.position.x - offset_odom.pose.pose.position.x
        new_odom.pose.pose.position.x = new_odom.pose.pose.position.x * 1.25 # offset piscina
        new_odom.pose.pose.position.y    =  -(msg.pose.pose.position.y - offset_odom.pose.pose.position.y)
        new_odom.pose.pose.position.y = new_odom.pose.pose.position.y * 1.25 # offset piscina
        new_odom.pose.pose.position.z    =  offset_z # teste na quadra
        #new_odom.pose.pose.position.z    =   msg.pose.pose.position.z - offset_odom.pose.pose.position.z
        new_odom.pose.pose.orientation.z =  -(msg.pose.pose.orientation.z)
        aux = new_odom.twist.twist.linear.x
        new_odom.twist.twist.linear.x    =  msg.twist.twist.linear.y 
        new_odom.twist.twist.linear.y    =  -(aux)
    
    pub_new_odom.publish(new_odom)

    if not is_valid_quaternion(new_odom.pose.pose.orientation):
        rospy.logwarn("Invalid quaternion")
        new_odom.pose.pose.orientation.x = last_x 
        new_odom.pose.pose.orientation.y = last_y 
        new_odom.pose.pose.orientation.z = last_z 
        new_odom.pose.pose.orientation.w = last_w  

    tf_broadcaster.sendTransform(
        (new_odom.pose.pose.position.x, new_odom.pose.pose.position.y, new_odom.pose.pose.position.z),
        (new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y, new_odom.pose.pose.orientation.z, new_odom.pose.pose.orientation.w),
        rospy.Time.now(),
        "base_link",  # child frame
        "world"       # parent frame
        )
    last_x = new_odom.pose.pose.orientation.x
    last_y = new_odom.pose.pose.orientation.y
    last_z = new_odom.pose.pose.orientation.z
    last_w = new_odom.pose.pose.orientation.w
    #print (status)

def is_valid_quaternion(orientation):
    """Checks if a quaternion is valid."""
    # Calculate the magnitude (length) of the quaternion
    length = math.sqrt(
        orientation.x ** 2 +
        orientation.y ** 2 +
        orientation.z ** 2 +
        orientation.w ** 2
    )
    # A valid quaternion should have a magnitude close to 1
    return math.isclose(length, 1.0, rel_tol=1e-3)

def main():
# cria node para o script
    rospy.init_node('aux_odom')
    
    pub_status_odom = rospy.Publisher('/tello/status_odom', Int32, queue_size=1)


# publicada a nova odometria
    pub_new_odom = rospy.Publisher('/tello/new_odom', Odometry, queue_size=1)

# recebe valores para cálculo da nova odometria    
    rospy.Subscriber('/tello/odom', Odometry, callback_odom, callback_args=(pub_new_odom, pub_status_odom))
    rospy.Subscriber('/tello/takeoff', Empty, callback_takeoff)
    rospy.Subscriber('/tello/land', Empty, callback_land)
    rospy.Subscriber('/tello/status', TelloStatus, callback_offset)
    # rospy.sleep(4)
    # global status
    # status = "decolando"
    rospy.spin()




if __name__ == '__main__':
    main()