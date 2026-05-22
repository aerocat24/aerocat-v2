import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

#limite bateria: battery_percentage: 10
#senha wifi tello 12345678

class RemoteControlJoyNode:
    
    def __init__(self):

        # Inicializa variáveis

        self.op_mode = 0


        self.joy_left_y = 0
        self.joy_left_x = 0
        self.button_R1 = 0
        self.button_L1 = 0
        self.button_triangle = 0
        self.button_x = 0
        self.button_L3 = 0
        self.button_R3 = 0
        self.button_select = 0
        self.button_start = 0

        self.last_state_button_L1 = 0
        self.last_state_button_R1 = 0
        self.last_state_button_x = 0
        self.last_state_button_triangle = 0
        self.last_state_button_L3 = 0
        self.last_state_button_R3 = 0
        self.last_state_button_select = 0
        self.last_state_button_start = 0

         # defini qual controle será usado
        self.PS3 = 1
        self.PS5 = 2
        self.control_type = self.PS3

        self.txt_init = """
                    PRESSIONE PARA TELEOPERAR O DRONE:
                        
            🎮  🕹️  (analógico esquerdo para cima)     : frente          🎮  △ : cima
            🎮  🕹️  (analógico esquerdo para baixo)    : trás            🎮  ✕ : baixo
            🎮  🕹️  (analógico esquerdo para direita)  : direita         🎮  R1 : girar sentido horário
            🎮  🕹️  (analógico esquerdo para esquerda) : esquerda        🎮  L1 : girar sentido anti-horário

            🎮  L3 : decolar         🎮  R3 : pousar

        """

        #velocidade do drone 0 a 1.0
        self.speed = 1.0

        #Publishers
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        self.pub_op_mode = rospy.Publisher('/tello/op_mode', Int32, queue_size=10)
        
        #Subscribers
        rospy.Subscriber("/tello/joy", Joy, self.joy_callback)
        rospy.Subscriber("/tello/op_mode", Int32, self.op_mode_callback)

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

    def op_mode_callback(self, msg):
        self.op_mode = msg.data
        
        # se estiver no modo manual
        if self.op_mode == 0:
            rospy.logwarn(f"modo manual ativado")
            # delay de 40ms apenas para garantir que o nó de controle não publique velocidades depois de ter mandado zerar as velocidades
            rospy.sleep(0.04)
            # zera todas as velocidades
            self.set_cmd_vel(0,0,0,0)
        elif self.op_mode == 1:
            rospy.logwarn(f"modo scan ativado")
        elif self.op_mode == 2:
            rospy.logwarn(f"modo pouso de precisão ativado")

    def joy_callback(self, data):

        if self.control_type == self.PS3:
            self.button_select = int(data.buttons[10])
            self.button_start = int(data.buttons[11])
        elif self.control_type == self.PS5:
            self.button_select = int(data.buttons[8])
            self.button_start = int(data.buttons[9])


        # garante que o select só funcione se o start não estiver pressionado
        if self.button_start == 0:
            # se o botão select for pressionado ()
            if self.button_select == 1 and self.last_state_button_select == 0:

                # se estiver no modo navegação autonoma
                if self.op_mode == 2:
                    self.op_mode = 0
                else:
                    # alterna o estado entre 0 e 1
                    # 0 manual, 1 pouso de precisão
                    self.op_mode = 1 - self.op_mode

                self.pub_op_mode.publish(self.op_mode)
                
                # se estiver no modo manual
                if self.op_mode == 0:
                    # zera todas as velocidades
                    self.set_cmd_vel(0,0,0,0)

        # atualiza o estado anterior        
        self.last_state_button_select = self.button_select

        # garante que o start só funcione se o select não estiver pressionado
        if self.button_select == 0:
            # se o botão start for pressionado
            if self.button_start == 1 and self.last_state_button_start == 0:

                # se estiver no modo de pouso de precisão
                if self.op_mode == 1:
                    self.op_mode = 0
                else:
                    # alterna o estado entre 0 e 2
                    # 0 manual, 2 navegação autonoma
                    self.op_mode = 2 - self.op_mode

                self.pub_op_mode.publish(self.op_mode)
                
                # se estiver no modo manual
                if self.op_mode == 0:
                    # zera todas as velocidades
                    self.set_cmd_vel(0,0,0,0)

        # atualiza o estado anterior        
        self.last_state_button_start = self.button_start

        if self.control_type == self.PS3:
            self.joy_left_x = float(data.axes[0])*1.0
            
            self.joy_left_y = float(data.axes[1])*1.0
            
            # subir
            self.button_triangle = int(data.buttons[4]) 

            # descer 
            self.button_x = int(data.buttons[0]) 

            # girar no sentido horário 
            self.button_R1 = int(data.buttons[7])
            # girar no sentido anti-horário
            self.button_L1 = int(data.buttons[6])
            # decolar
            self.button_L3 = int(data.buttons[13])
            # pousar
            self.button_R3 = int(data.buttons[14])
        elif self.control_type == self.PS5:
            self.joy_left_x = float(data.axes[0])*1.0
            
            self.joy_left_y = float(data.axes[1])*1.0
            
            # subir
            self.button_triangle = int(data.buttons[2]) 

            # descer 
            self.button_x = int(data.buttons[0]) 

            # girar no sentido horário 
            self.button_R1 = int(data.buttons[5])
            # girar no sentido anti-horário
            self.button_L1 = int(data.buttons[4])
            # decolar
            self.button_L3 = int(data.buttons[11])
            # pousar
            self.button_R3 = int(data.buttons[12])


        vlx = -self.joy_left_x 
        vly = self.joy_left_y 

        # subir
        if self.button_triangle == 1:
            vlz = self.speed
        # descer
        elif self.button_x == 1:
            vlz = -self.speed
        else:
            vlz = 0

        # girar no sentido horário
        if self.button_R1 == 1:
            vaz = self.speed
        # girar no sentido anti-horário
        elif self.button_L1 == 1:
            vaz = -self.speed
        else:
            vaz = 0

        self.set_cmd_vel(vlx,vly,vlz,vaz)
        
        
        # decolar
        if self.button_L3 == 1 and self.last_state_button_L3 == 0:
            self.takeoff()

        # pousar
        if self.button_R3 == 1 and self.last_state_button_R3 == 0:
            self.land()

        self.last_state_button_L3 = self.button_L3 
        self.last_state_button_R3 = self.button_R3 


if __name__ == '__main__':

    rospy.init_node('remote_control_joy_node')

    # inicializa como None para evitar erro no finally
    remote_control = None  

    def shutdown_hook():
        """ Função para ficar parado ao encerrar o nó. """
        if remote_control is not None:
            remote_control.set_cmd_vel(0,0,0,0)

    #associa a função para ser chamada no encerramento
    rospy.on_shutdown(shutdown_hook)  

    try:
        remote_control = RemoteControlJoyNode()

        # loop para manter o nó ativo sem precisar do rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(1)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

    finally:
        pass