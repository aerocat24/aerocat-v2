import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

class RemoteControlJoyNode:

    def __init__(self):

        rospy.init_node('remote_control_joy_node', anonymous=True)
        # Inicializa variáveis
        self.op_mode = 0

        self.button_L1 = 0
        self.button_R1 = 0
        self.button_L2 = 0
        self.button_R2 = 0
        self.button_select = 0
        self.last_state_button_L1 = 0
        self.last_state_button_R1 = 0
        self.last_state_button_L2 = 0
        self.last_state_button_R2 = 0
        self.last_state_button_select = 0
        self.next_point = 0
        self.joy_left = 0.0
        self.joy_right = 0.0
        self.pot = 0.3
        self.pwm_left = 0.0
        self.pwm_right = 0.0

        # defini qual controle será usado
        self.PS3 = 1
        self.PS5 = 2
        self.control_type = self.PS3

        self.txt_init = """
                   PRESSIONE PARA TELEOPERAR O BARCO:
                        
            🎮  🕹️  (analógico esquerdo para cima)  : direita  
            🎮  🕹️  (analógico direito para cima)   : esquerda   

            🎮  L1 : diminuir potência máxima  
            🎮  R1 : aumentar potência máxima  

            🎮  select : alternar entre modo manual/autônomo 
        """


        # Publishers
        self.pub_pwm = rospy.Publisher('/boat/pwm', Int32MultiArray, queue_size=10)
        self.pub_op_mode = rospy.Publisher('/boat/op_mode', Int32, queue_size=10)
        self.pub_next_point = rospy.Publisher('/boat/next_point', Int32, queue_size=10)

        # Subscribers
        rospy.Subscriber("/boat/joy", Joy, self.joy_callback)
        rospy.Subscriber("/boat/op_mode", Int32, self.op_mode_callback)

        print(self.txt_init)

        rospy.on_shutdown(self.shutdown_hook)  

    def shutdown_hook(self):
        # zera os pwm
        pwm = Int32MultiArray()
        pwm.data = [0, 0]
        self.pub_pwm.publish(pwm)

    def op_mode_callback(self, msg):
        self.op_mode = msg.data



        # se estiver no modo manual
        if self.op_mode == 0:
            rospy.logwarn(f"modo manual ativado")
            # delay de 40ms apenas para garantir que o nó de controle não publique pwm depois de ter mandado zerar o pwm
            rospy.sleep(0.04)
            # zera os pwm
            pwm = Int32MultiArray()
            pwm.data = [0, 0]
            self.pub_pwm.publish(pwm)
        else:
            rospy.logwarn(f"modo auto ativado")

    def joy_callback(self, data):

        if self.control_type == self.PS3:
            # Atualiza os valores dos botões e eixos do controle
            self.button_L1 = int(data.buttons[6])
            self.button_R1 = int(data.buttons[7])
            self.button_L2 = int(data.buttons[8])
            self.button_R2 = int(data.buttons[9])
            self.button_select = int(data.buttons[10])

            # verifica se o valor joy left é maior que zero
            if float(data.axes[1]) > 0:
                # define o valor maximo
                self.joy_left = float(data.axes[1])
            else:
                # define o valor zero
                self.joy_left = 0.0

            # verifica se o valor joy right é maior que zero
            if float(data.axes[3]) > 0:
                # define o valor maximo
                self.joy_right = float(data.axes[3])
            else:
                # define o valor zero   
                self.joy_right = 0.0  
        elif self.control_type == self.PS5:
            # Atualiza os valores dos botões e eixos do controle
            self.button_L1 = int(data.buttons[4])
            self.button_R1 = int(data.buttons[5])
            self.button_L2 = int(data.buttons[6])
            self.button_R2 = int(data.buttons[7])
            self.button_select = int(data.buttons[8])

            # verifica se o valor joy left é maior que zero
            if float(data.axes[1]) > 0:
                # define o valor maximo
                self.joy_left = float(data.axes[1])
            else:
                # define o valor zero
                self.joy_left = 0.0

            # verifica se o valor joy right é maior que zero
            if float(data.axes[4]) > 0:
                # define o valor maximo
                self.joy_right = float(data.axes[4])
            else:
                # define o valor zero   
                self.joy_right = 0.0  

    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s 

        while not rospy.is_shutdown():

            if self.button_select == 1 and self.last_state_button_select == 0:
                # alterna o estado entre 0 e 1
                # 0 manual, 1 auto
                self.op_mode = 1 - self.op_mode
                self.pub_op_mode.publish(self.op_mode)
                
                #se estiver no modo manual
                if self.op_mode == 0:
                    #zera os pwm
                    pwm = Int32MultiArray()
                    pwm.data = [0, 0]
                    self.pub_pwm.publish(pwm)

            # atualiza o estado anterior        
            self.last_state_button_select = self.button_select

            if self.button_L2 == 1 and self.last_state_button_L2 == 0:
                self.next_point = self.button_L2-1
                rospy.loginfo(f"next_point: {self.next_point}")
                self.pub_next_point.publish(self.next_point)
            
            # atualiza o estado anterior        
            self.last_state_button_L2 = self.button_L2

            if self.button_R2 == 1 and self.last_state_button_R2 == 0:
                self.next_point = self.button_R2
                rospy.loginfo(f"next_point: {self.next_point}")
                self.pub_next_point.publish(self.next_point)
            
            # atualiza o estado anterior        
            self.last_state_button_R2 = self.button_R2
            
            # se estiver no modo manual
            if self.op_mode == 0:
                # Verifica se houve mudança no estado do botão L1
                if self.button_L1 != self.last_state_button_L1:
                    self.last_state_button_L1 = self.button_L1

                    if self.button_L1 == 1:
                        if self.pot > 0.4:
                            self.pot -= 0.1
                        else:
                            self.pot = 0.3

                # Verifica se houve mudança no estado do botão L2
                if self.button_R1 != self.last_state_button_R1:
                    self.last_state_button_R1 = self.button_R1

                    if self.button_R1 == 1:
                        if self.pot < 0.9:
                            self.pot += 0.1
                        else:
                            self.pot = 1.0

                # Calcula os valores PWM para os motores
                self.pwm_left = 180.0 * self.joy_left * self.pot
                self.pwm_right = 180.0 * self.joy_right * self.pot

                # Garantir que o PWM não seja negativo
                if self.pwm_left < 0:
                    self.pwm_left = 0
                if self.pwm_right < 0:
                    self.pwm_right = 0

                pwm = Int32MultiArray()
                pwm.data = [int(self.pwm_left), int(self.pwm_right)]
                #rospy.loginfo(pwm)
                #log = f"Publish pwm_left: {pwm.data[0]} pwm_right: {pwm.data[1]}"
                
                #rospy.loginfo(log)
                self.pub_pwm.publish(pwm)

            rate.sleep()


if __name__ == '__main__':

    try:
        remote_control_node = RemoteControlJoyNode()
        remote_control_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass
