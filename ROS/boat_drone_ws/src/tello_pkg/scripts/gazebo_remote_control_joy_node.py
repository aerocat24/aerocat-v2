import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class GazeboRemoteControlJoyNode:
    def __init__(self):
        # Inicializa variáveis
        self.button_up_down_arrow = 0
        self.button_up_arrow = 0
        self.button_down_arrow = 0
        self.button_right_left_arrow = 0
        self.button_right_arrow = 0
        self.button_left_arrow = 0
        self.button_triangle = 0
        self.button_circle = 0
        self.button_square = 0
        self.button_x = 0
        self.button_L3 = 0
        self.button_R3 = 0

        self.last_state_button_up_arrow = 0
        self.last_state_button_down_arrow = 0
        self.last_state_button_right_arrow = 0
        self.last_state_button_left_arrow = 0
        self.last_state_button_triangle = 0
        self.last_state_button_circle = 0
        self.last_state_button_square = 0
        self.last_state_button_x = 0
        self.last_state_button_L3 = 0
        self.last_state_button_R3 = 0

        self.txt_init = """
                    PRESSIONE PARA TELEOPERAR O DRONE:
                        
            🎮  ⬆️  : frente          🎮  △ : cima
            🎮  ⬇️  : trás            🎮  ✕ : baixo
            🎮  ➡️  : direita         🎮  ○ : girar sentido horário
            🎮  ⬅️  : esquerda        🎮  ▢ : girar sentido anti-horário

            🎮  L3 : decolar         🎮  R3 : pousar

        """

        #velocidade do drone 0 a 1.0
        self.speed = 0.50

        #Publishers
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        
        #Subscribers
        rospy.Subscriber("/joy", Joy, self.joy_callback)

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

    def land(self):
        self.pub_land.publish()
    
    def takeoff(self):
        self.pub_takeoff.publish()

    def joy_callback(self, data):

        #zera todas as velocidades
        vlx = 0 #velocidade linear no eixo x
        vly = 0 #velocidade linear no eixo y
        vlz = 0 #velocidade linear no eixo z
        vaz = 0 #velocidade angular no eixo z

        # Atualiza os valores dos botões e eixos do controle
        self.button_up_down_arrow = int(data.axes[5])
        self.button_right_left_arrow = int(data.axes[4])

        if self.button_up_down_arrow != 0:
            if self.button_up_down_arrow > 0:
                # frente
                self.button_up_arrow = 1
                self.button_down_arrow = 0
            else:
                # trás
                self.button_down_arrow = 1
                self.button_up_arrow = 0
        else:
            self.button_up_arrow = 0
            self.button_down_arrow = 0

        if self.button_right_left_arrow != 0:

            if self.button_right_left_arrow > 0:
                # esquerda
                self.button_left_arrow = 1
                self.button_right_arrow = 0
            else:
                # direita
                self.button_right_arrow = 1
                self.button_left_arrow = 0
        else:
            self.button_right_arrow = 0
            self.button_left_arrow = 0

        # subir
        self.button_triangle = int(data.buttons[0])
        # descer 
        self.button_x = int(data.buttons[2])
        # girar no sentido horário 
        self.button_circle = int(data.buttons[1])
        # girar no sentido anti-horário
        self.button_square = int(data.buttons[3])
        # decolar
        self.button_L3 = int(data.buttons[10])
        # pousar
        self.button_R3 = int(data.buttons[11])

        # frente
        if self.button_up_arrow != self.last_state_button_up_arrow:

            self.last_state_button_up_arrow = self.button_up_arrow

            if self.button_up_arrow == 1:
                vlx = self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_up_arrow == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        # trás
        elif self.button_down_arrow != self.last_state_button_down_arrow:

            self.last_state_button_down_arrow = self.button_down_arrow

            if self.button_down_arrow == 1:
                vlx = -self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_down_arrow == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)
        
        # esquerda
        elif self.button_left_arrow != self.last_state_button_left_arrow:

            self.last_state_button_left_arrow = self.button_left_arrow

            if self.button_left_arrow == 1:
                vly = self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_left_arrow == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        # direita
        elif self.button_right_arrow != self.last_state_button_right_arrow:

            self.last_state_button_right_arrow = self.button_right_arrow

            if self.button_right_arrow == 1:
                vly = -self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_right_arrow == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        # subir
        elif self.button_triangle != self.last_state_button_triangle:

            self.last_state_button_triangle = self.button_triangle

            if self.button_triangle == 1:
                vlz = self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_triangle == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        # descer
        elif self.button_x != self.last_state_button_x:

            self.last_state_button_x = self.button_x

            if self.button_x == 1:
                vlz = -self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_x == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        # girar no sentido horário
        elif self.button_circle != self.last_state_button_circle:

            self.last_state_button_circle = self.button_circle

            if self.button_circle == 1:
                vaz = -self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_circle == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)
        

        # girar no sentido anti-horário
        elif self.button_square != self.last_state_button_square:

            self.last_state_button_square = self.button_square

            if self.button_square == 1:
                vaz = self.speed
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            elif self.button_square == 0:
                self.set_cmd_vel(vlx,vly,vlz,vaz)

        
        # decolar
        elif self.button_L3 == 1 and self.last_state_button_L3 == 0:
            self.takeoff()

        # pousar
        elif self.button_R3 == 1 and self.last_state_button_R3 == 0:
            self.land()

        self.last_state_button_L3 = self.button_L3 
        self.last_state_button_R3 = self.button_R3 

            
if __name__ == '__main__':

    rospy.init_node('gazebo_remote_control_joy_node')

    # inicializa como None para evitar erro no finally
    gazebo_remote_control = None  

    def shutdown_hook():
        """ Função para ficar parado ao encerrar o nó. """
        if gazebo_remote_control is not None:
            gazebo_remote_control.set_cmd_vel(0,0,0,0)

    # Registra a função para ser chamada no encerramento
    rospy.on_shutdown(shutdown_hook)  

    try:
        gazebo_remote_control = GazeboRemoteControlJoyNode()

        # Loop para manter o nó ativo sem precisar do rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(1)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

    finally:
        pass
            
