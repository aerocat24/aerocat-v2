import rospy
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class PositionDynamicNode:

    def __init__(self):

        rospy.init_node('position_dynamic_node', anonymous=True)

        # Inicializando os valores recebidos

        self.last_point_time = None
        
        self.compass_data = None
        self.gps_data = None
        self.current_theta = 0
        self.gps_lat = 0.0
        self.gps_lng = 0.0
        self.op_mode = 0  #0 manual 1 auto
        self.point_reached = 0 #0 ainda nao chegou no ponto, 1 ja chegou
        #primeiro ponto
        self.current_point = 0

        self.timer_active = False
        
        # inicia os vetores de posição com lista vazia
        self.destination_lat = []
        self.destination_lng = []

        #Subscribers
        rospy.Subscriber("/boat/compass", Int32, self.compass_callback)
        rospy.Subscriber("/boat/gps", Float32MultiArray, self.gps_callback)
        rospy.Subscriber("/boat/op_mode", Int32, self.op_mode_callback)

        #Publishers
        self.pub_distance = rospy.Publisher('/boat/distance', Float32, queue_size=10)
        self.pub_theta = rospy.Publisher('/boat/theta', Float32, queue_size=10)
        self.pub_delta_theta = rospy.Publisher('/boat/delta_theta', Float32, queue_size=10)
        self.pub_op_mode = rospy.Publisher('/boat/op_mode', Int32, queue_size=10)
        self.pub_pwm = rospy.Publisher('/boat/pwm', Int32MultiArray, queue_size=10)
        self.pub_point_reached = rospy.Publisher('/boat/point_reached', Int32, queue_size=10)


    def compass_callback(self, msg):
        if self.compass_data is None:
            self.compass_data = msg.data #compensaçao

            self.current_theta = self.compass_data


    def gps_callback(self, msg):
        if self.gps_data is None:
            self.gps_data = msg.data
            self.gps_lat = self.gps_data[0]
            self.gps_lng = self.gps_data[1]

    def pos_marker_callback(self, msg):
        # adiciona o novo ponto recebido na lista de pontos de destino
        self.destination_lat.append(msg.data[0])
        self.destination_lng.append(msg.data[1])


    def op_mode_callback(self, msg):
        self.op_mode = msg.data


    def _distance_between(self, lat1, lon1, lat2, lon2):

        R = 6371009  # Raio da Terra em metros
        # Converter graus para radianos
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Diferenças entre os pontos
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Fórmula de haversine
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def _course_to(self, lat1, lon1, lat2, lon2):
        # Converter graus para radianos
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Diferença de longitude
        dlon = lon2 - lon1
        
        # Fórmula do azimute
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        initial_bearing = math.atan2(x, y)
        
        # Converter de radianos para graus e ajustar para 0°-360°
        return (math.degrees(initial_bearing) + 360) % 360
    
    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s

        while not rospy.is_shutdown():

            # se estiver no modo auto e se ainda nao estiver alcançado o ponto desejado
            if self.op_mode == 1 and self.point_reached == 0:

                # se tiver chegado valor da bussula e gps e ainda tiver pontos na lista
                if self.compass_data is not None and self.gps_data is not None and len(self.destination_lat) > 0 and len(self.destination_lng) > 0:
                    # rospy.loginfo(f"compass: {self.current_theta}, Lat: {self.gps_lat}, Log: {self.gps_lng}")
                    
                    # ângulo para o ponto de referência
                    theta = self._course_to(self.gps_lat, self.gps_lng, self.destination_lat[self.current_point], self.destination_lng[self.current_point])
                    # distância para o ponto de referência
                    distance = self._distance_between(self.gps_lat, self.gps_lng, self.destination_lat[self.current_point], self.destination_lng[self.current_point])

                    # variação do ângulo
                    delta_theta = theta - self.current_theta

                    # caso o angulo da bussula estiver tendo pequenas variações
                    # está 0 por enquanto
                    # if(abs(delta_theta) <= 10): 
                    #    delta_theta = 0

                    rospy.loginfo(f"")
                    rospy.logwarn(f"ponto destino: {self.current_point+1}")
                    rospy.loginfo(f"theta: {theta}")
                    rospy.loginfo(f"current_theta: {self.current_theta}")
                    rospy.loginfo(f"delta_theta: {delta_theta}")
                    rospy.loginfo(f"distance: {distance}")

                    # se o barco estiver em um raio de 2.5m
                    if(distance <= 2.5):
                        rospy.logwarn(f"Chegou no ponto {self.current_point+1}")
                        rospy.logwarn(f"Aguardando o proximo ponto...")
                        # alcançou o ponto desejado
                        self.point_reached = 1
                        # publica que chegou no ponto
                        self.pub_point_reached.publish(self.point_reached)
                        # delay de 40ms apenas para garantir que o nó de controle não publique pwm depois de ter mandado zerar o pwm
                        rospy.sleep(0.04)
                        # zera os pwm
                        pwm = Int32MultiArray()
                        pwm.data = [0, 0]
                        self.pub_pwm.publish(pwm)
                    
                    # publica os tópicos
                    self.pub_theta.publish(theta)
                    self.pub_distance.publish(distance)
                    self.pub_delta_theta.publish(delta_theta)

                    # limpa as variaveis
                    self.compass_data = None
                    self.gps_data = None
                    
            # se estiver no modo manual
            else:
                # limpa as variaveis
                self.compass_data = None
                self.gps_data = None

            # se estiver o ponto estiver alcançado e o timer estiver definido
            if self.point_reached == 1 and self.last_point_time is None: 
                self.last_point_time = rospy.Time.now()
                self.current_point += 1
                self.timer_active = False

            # aguarda no mínimo 10s antes de ir para o próximo ponto
            elif (rospy.Time.now() - self.last_point_time).to_sec() >= 10:

                if self.timer_active is False:
                    self.timer_active = True
                    # já esperou 10s
                    rospy.logwarn("Passou 10s de espera, proximo ponto liberado")

                if self.current_point < len(self.destination_lng):
                    rospy.logwarn("Resetando o timer e indo para o próximo ponto")
                    # reseta o timer
                    self.last_point_time = None
                    # reseta a variavel 
                    self.point_reached = 0


            rate.sleep()


if __name__ == '__main__':

    try:
        position_dynamic = PositionDynamicNode()
        position_dynamic.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass
