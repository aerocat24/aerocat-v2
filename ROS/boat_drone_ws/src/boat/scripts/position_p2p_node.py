import rospy
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class PositionP2PNode:

    def __init__(self):
        
        rospy.init_node("position_p2p_node", anonymous=False)

        # Inicializando os valores recebidos
        
        self.compass_data = None
        self.gps_data = None
        self.current_theta = 0
        self.gps_lat = 0.0
        self.gps_lng = 0.0
        self.op_mode = 0 # 0 manual 1 auto 

        # coordenadas de origem fixa do sistema (piscina)
        self.lat0 = -21.532712936401367 
        self.lon0 = -42.63380813598633 

        #angulo de rotação do sistema
        self.theta_deg = 136 
        self.theta = math.radians(self.theta_deg)

        # fator de conversão aproximado
        self.meters_per_degree = 111319.9

        #primeiro ponto
        self.current_point = 0
    
        # waypoints pré definidos em xy (metros)
        # self.x_sp = [8.0, 8.0, 4.5, 4.5, 8.0]  
        # self.y_sp = [5.0, 15.0, 15.0, 5.0, 5.0]

        self.x_sp = [4.5]  
        self.y_sp = [15.0]

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
        self.pub_pose_xy = rospy.Publisher('/boat/pose_xy', Float32MultiArray, queue_size=10)


    def compass_callback(self, msg):
        if self.compass_data is None:
            self.compass_data = msg.data 
            # rospy.loginfo(f"compass recebido: {self.compass_dat}")
            self.offset = 150 # compensaçao
            self.current_theta = -(self.compass_data + self.offset)

            if(self.current_theta > 180):
                self.current_theta = self.current_theta - 360
            elif(self.current_theta < -180):
                self.current_theta = self.current_theta + 360

            rospy.loginfo(f"theta atual: {self.current_theta}")


    def gps_callback(self, msg):
        if self.gps_data is None:
            self.gps_data = msg.data
            self.gps_lat = self.gps_data[0]
            self.gps_lng = self.gps_data[1]
            # rospy.loginfo(f"gps recebido: lat {self.gps_lat}, lng {self.gps_lng}")

            # converter gps para xy
            x = (self.gps_lng  - self.lon0) * self.meters_per_degree * math.cos(
                math.radians((self.lat0 + self.gps_lat) / 2))
            
            y = (self.gps_lat - self.lat0) * self.meters_per_degree

            # rotação do sitema, para ficar no mesmo eixo da piscina
            self.x_rot = x * math.cos(self.theta) - y * math.sin(self.theta)
            self.y_rot = x * math.sin(self.theta) + y * math.cos(self.theta)

            rospy.loginfo(f"pose recebido: x {self.x_rot:.2f} m, y {self.y_rot:.2f} m")

            pose_xy = Float32MultiArray()
            pose_xy.data = [self.x_rot, self.y_rot]
            self.pub_pose_xy.publish(pose_xy)


    def op_mode_callback(self, msg):
        self.op_mode = msg.data


    def run(self):
     
        rate = rospy.Rate(10) #10Hz -> 0.1s 

        while not rospy.is_shutdown():

            #se estiver no modo auto 
            if self.op_mode == 1:
                # verifica se chegou um novo marcador de destino
                if(self.current_point < len(self.x_sp)):
                    #se se tiver chegado valor da bussula e gps
                    if self.compass_data is not None and self.gps_data is not None:

                        # calcula a diferença entre a posição desejada e a posição atual
                        dx = self.x_sp[self.current_point] - self.x_rot
                        dy = self.y_sp[self.current_point] - self.y_rot

                        # ângulo para o ponto de referência
                        theta_rad = math.atan2(dy, dx)
                        # converte para graus
                        theta = math.degrees(theta_rad)

                        # distância linear para o ponto de referência
                        distance = math.sqrt(dx**2 + dy**2)

                        # variação do ângulo
                        delta_theta = theta - self.current_theta

                        #ajuste do delta_theta para ficar entre -180 e 180
                        if delta_theta > 180:
                            delta_theta -= 360
                        elif delta_theta < -180:
                            delta_theta += 360

                        rospy.loginfo(f"")

                        # se o ponto for o último
                        if(len(self.x_sp) - 1 == self.current_point):
                            rospy.logwarn(f"indo para o destino final")
                        else:
                            rospy.logwarn(f"indo para o marcador {self.current_point+1}")

                        rospy.loginfo(f"theta: {theta}")
                        rospy.loginfo(f"current_theta: {self.current_theta}")
                        rospy.loginfo(f"delta_theta: {delta_theta}")
                        rospy.loginfo(f"distance: {distance}")

                        #se o barco estiver em um raio de 2.5m
                        if(distance <= 2.5):
                            # se o ponto for o último
                            if(len(self.x_sp) - 1 == self.current_point):
                                rospy.logwarn(f"Chegou no destino final")
                                rospy.logwarn(f"Modo manual ativado")
                                self.current_point = 0
                                self.end_sp = False
                                self.op_mode = 0
                                self.pub_op_mode.publish(self.op_mode)
                                # delay de 40ms apenas para garantir que o nó de controle não publique pwm depois de ter mandado zerar o pwm
                                rospy.sleep(0.04)
                                # zera os pwm
                                pwm = Int32MultiArray()
                                pwm.data = [0, 0]
                                self.pub_pwm.publish(pwm)
                            else:
                                rospy.logwarn(f"Chegou no waypoint {self.current_point+1}")
                            
                            self.current_point += 1
                        
                        # publica os tópicos
                        self.pub_theta.publish(theta)
                        self.pub_distance.publish(distance)
                        self.pub_delta_theta.publish(delta_theta)

                        # limpa as variaveis
                        self.compass_data = None
                        self.gps_data = None
                    
            # se estiver no modo manual
            else:
                #limpa as variaveis
                self.compass_data = None
                self.gps_data = None

            rate.sleep()


if __name__ == '__main__':

    try:
        position_p2p_node = PositionP2PNode()
        position_p2p_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass
