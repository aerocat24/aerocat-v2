import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray

class ControlM2MNode:

    def __init__(self):
        # Inicializando os valores recebidos
        self.distance_data = None
        self.delta_theta_data = None
        #distance entre os propulsores (metros)
        self.L = 0.50 
        #ganhos
        self.Kv = 0.1    #N/m
        self.Kt = 0.2233 #N.m/rad
        self.Km = 0.0671 #N/PWM

        self.pwm_min = 60
        self.pwm_max = 120

        self.op_mode = 0  #0 manual 1 auto 
        self.marker_reached = 0 #0 ainda nao chegou no ponto, 1 ja chegou

        #Subscribers
        rospy.Subscriber("/boat/distance", Float32, self.distance_callback)
        rospy.Subscriber("/boat/delta_theta", Float32, self.delta_theta_callback)
        rospy.Subscriber("/boat/op_mode", Int32, self.op_mode_callback)
        rospy.Subscriber("/marker_reached", Int32, self.marker_reached_callback)

        #Publishers
        self.pub_pwm = rospy.Publisher('/boat/pwm', Int32MultiArray, queue_size=10)

    def distance_callback(self, msg):
        if self.distance_data is None:
            self.distance_data = msg.data
            self.process_data()

    def delta_theta_callback(self, msg):
        if self.delta_theta_data is None:
            self.delta_theta_data = msg.data
            self.process_data()

    def op_mode_callback(self, msg):
        self.op_mode = msg.data

    def marker_reached_callback(self, msg):
        self.marker_reached = msg.data


    def process_data(self):

        #se estiver no modo auto e se ainda nao estiver alcançado o marcador desejado
        if self.op_mode == 1 and self.marker_reached == 0:

            if self.distance_data is not None and self.delta_theta_data is not None:
                rospy.loginfo(f"Received: distance: {self.distance_data}, delta_theta: {self.delta_theta_data}")

                d = self.distance_data
                delta_theta = self.delta_theta_data

                #força calculada para o movimento frontal do barco
                Fv = self.Kv*d  # [10 N] = kv*[10m]

                #Torque necessário para a embarcação se orientar em direção ao ponto
                Fw = self.Kt*delta_theta # [10 N.m] = kt*[pi rad?]

                #força para direita 
                Fr = Fv + (self.L*Fw)/2.0
                
                if(Fr < 0):
                    Fr = 0

                #força para esquerda 
                Fl = Fv - (self.L*Fw)/2.0

                if(Fl < 0):
                    Fl = 0

                #conversão das forças para PWMs
                pwm_left = self.pwm_min + Fl/self.Km
                pwm_right = self.pwm_min  + Fr/self.Km

                #saturação
                if(pwm_left > self.pwm_max):
                    pwm_left = self.pwm_max
                
                if(pwm_right > self.pwm_max):
                    pwm_right = self.pwm_max

                pwm = Int32MultiArray()
                pwm.data = [int(pwm_left), int(pwm_right)]
    
                log = f"Publish: pwm_left: {pwm.data[0]} pwm_right: {pwm.data[1]}"
                
                rospy.loginfo(log)
                self.pub_pwm.publish(pwm)

                #limpa as variaveis
                self.distance_data = None
                self.delta_theta_data = None
        #se estiver no modo manual
        else:
            #limpa as variaveis
            self.distance_data = None
            self.delta_theta_data = None

if __name__ == '__main__':

    rospy.init_node('control_m2m_node')

    #Inicia o objeto
    control_m2m_node = ControlM2MNode()

    # Mantém o nó ativo
    rospy.spin()
