import rospy
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class PositionNode:

    def __init__(self):
        # Inicializando os valores recebidos
        
        self.compass_data = None
        self.gps_data = None
        self.current_theta = 0
        self.gps_lat = 0.0
        self.gps_lng = 0.0
        self.op_mode = 0 #0 manual 1 auto
        #primeiro ponto
        self.current_point = 0
    
        #vetor com os n pontos 
        self.destination_lat = [-21.532611460]
        # longitude
        self.destination_lng = [-42.633832481]

        #Subscribers
        rospy.Subscriber("/boat/compass", Int32, self.compass_callback)
        rospy.Subscriber("/boat/gps", Float32MultiArray, self.gps_callback)
        rospy.Subscriber("/boat/op_mode", Int32, self.op_mode_callback)

        #Publishers
        self.pub_distance = rospy.Publisher('/boat/distance', Float32, queue_size=10)
        self.pub_theta = rospy.Publisher('/boat/theta', Float32, queue_size=10)
        self.pub_delta_theta = rospy.Publisher('/boat/delta_theta', Float32, queue_size=10)
        self.pub_op_mode = rospy.Publisher('/boat/op_mode', Int32, queue_size=10)


    def compass_callback(self, msg):
        if self.compass_data is None:
            self.compass_data = msg.data #compensaçao

            self.current_theta = self.compass_data
            self.process_data()

    def gps_callback(self, msg):
        if self.gps_data is None:
            self.gps_data = msg.data
            self.gps_lat = self.gps_data[0]
            self.gps_lng = self.gps_data[1]
            self.process_data()

    def op_mode_callback(self, msg):
        self.op_mode = msg.data

    def process_data(self):

        #se estiver no modo auto 
        if self.op_mode == 1:

            #se se tiver chegado valor da bussula e gps
            if self.compass_data is not None and self.gps_data is not None:
                #rospy.loginfo(f"compass: {self.current_theta}, Lat: {self.gps_lat}, Log: {self.gps_lng}")
                
                #ângulo para o ponto de referência
                theta = self._course_to(self.gps_lat, self.gps_lng, self.destination_lat[self.current_point], self.destination_lng[self.current_point])
                #distância para o ponto de referência
                distance = self._distance_between(self.gps_lat, self.gps_lng, self.destination_lat[self.current_point], self.destination_lng[self.current_point])

                #variação do ângulo
                delta_theta = theta - self.current_theta


                #ajuste do delta_theta para ficar entre -180 e 180
                if delta_theta > 180:
                    delta_theta -= 360
                elif delta_theta < -180:
                    delta_theta += 360

                #caso o angulo da bussula estiver tendo pequenas variações
                #está 0 por enquanto
                #if(abs(delta_theta) <= 10): 
                #    delta_theta = 0

                rospy.loginfo(f"")
                rospy.logwarn(f"ponto destino: {self.current_point+1}")
                rospy.loginfo(f"theta: {theta}")
                rospy.loginfo(f"current_theta: {self.current_theta}")
                rospy.loginfo(f"delta_theta: {delta_theta}")
                rospy.loginfo(f"distance: {distance}")

                #se o barco estiver em um raio de 2.5m
                if(distance <= 2.5):
                    rospy.logwarn(f"Chegou no ponto {self.current_point+1}")
                    self.current_point += 1
                    #se ja passou pelo ultimo
                    if self.current_point > (len(self.destination_lat)-1): 
                        #troca para o modo manual
                        rospy.logwarn(f"Chegou no ultimo ponto")
                        rospy.logwarn(f"Modo manual ativado")
                        self.current_point = 0
                        self.op_mode = 0
                        self.pub_op_mode.publish(self.op_mode)
                
                #publica os tópicos
                self.pub_theta.publish(theta)
                self.pub_distance.publish(distance)
                self.pub_delta_theta.publish(delta_theta)

                #limpa as variaveis
                self.compass_data = None
                self.gps_data = None
                
        #se estiver no modo manual
        else:
            #limpa as variaveis
            self.compass_data = None
            self.gps_data = None


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
        
        # Converter de radianos para graus e ajustar para -180° a 180°
        return ((math.degrees(initial_bearing) + 180) % 360) - 180


if __name__ == '__main__':

    rospy.init_node('position_node')

    #Inicia o objeto
    position_node = PositionNode()

    # Mantém o nó ativo
    rospy.spin()
