import rospy
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class PositionM2MNode:

    def __init__(self):
        
        rospy.init_node("position_m2m_node", anonymous=False)

        # Inicializando os valores recebidos
        
        self.compass_data = None
        self.gps_data = None
        self.current_theta = 0
        self.gps_lat = 0.0
        self.gps_lng = 0.0
        self.op_mode = 0 # 0 manual 1 auto 
        self.op_mode_tello = 0 # 0 manual 2 pouso de precisão

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
    
        # inicializa as listas de destino gps como vazias
        self.x_sp = []  
        self.y_sp = []
        # posição x e y de fim de inspeção 
        self.end_x = 11
        self.end_y = 22.5
        self.end_sp = False # verifica se é o destino final
        self.marker_id = []

        # Controle de tempo de espera
        self.waiting = False
        self.wait_start_time = None
        self.wait_duration = rospy.Duration(10.0)  # 10 segundos

        #Subscribers
        rospy.Subscriber("/boat/compass", Int32, self.compass_callback)
        rospy.Subscriber("/boat/gps", Float32MultiArray, self.gps_callback)
        rospy.Subscriber("/pos_xy_marker", Float32MultiArray, self.pos_xy_marker_callback)
        rospy.Subscriber("/boat/op_mode", Int32, self.op_mode_callback)
        rospy.Subscriber("/tello/op_mode", Int32, self.op_mode_tello_callback)

        #Publishers
        self.pub_distance = rospy.Publisher('/boat/distance', Float32, queue_size=10)
        self.pub_theta = rospy.Publisher('/boat/theta', Float32, queue_size=10)
        self.pub_delta_theta = rospy.Publisher('/boat/delta_theta', Float32, queue_size=10)
        self.pub_op_mode = rospy.Publisher('/boat/op_mode', Int32, queue_size=10)
        self.pub_pwm = rospy.Publisher('/boat/pwm', Int32MultiArray, queue_size=10)
        self.pub_pose_xy = rospy.Publisher('/boat/pose_xy', Float32MultiArray, queue_size=10)
        self.pub_marker_reached = rospy.Publisher('/marker_reached', Int32, queue_size=10)


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
        # self.compass_data = None
       


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
        # self.gps_data = None


    def pos_xy_marker_callback(self, msg):
        # recebe um novo ponto de destino em xy do marcador
        self.marker_id.append(msg.data[0])
        x, y = msg.data[1], msg.data[2]
        self.x_sp.append(x)
        self.y_sp.append(y)
        rospy.loginfo(f"Marcador id {self.marker_id[-1]:.0f} recebido")
        rospy.loginfo(f"Posição destino adicionada: {self.x_sp[-1]:.6f}, {self.y_sp[-1]:.6f}")


    def op_mode_callback(self, msg):
        self.op_mode = msg.data

    def op_mode_tello_callback(self, msg):
        self.op_mode_tello = msg.data
        if self.op_mode_tello == 2:
            # se já terminou a inspeção, adiciona o ponto final para o barco
            self.x_sp.append(self.end_x)
            self.y_sp.append(self.end_y)

    def run(self):
     
        rate = rospy.Rate(10) #10Hz -> 0.1s 

        while not rospy.is_shutdown():

            #se estiver no modo auto 
            if self.op_mode == 1:

                # Se estiver esperando 10s, verifica se já acabou o tempo
                if self.waiting:
                    elapsed = rospy.Time.now() - self.wait_start_time
                    rospy.logwarn(f"tempo: {elapsed.to_sec():.2f}s")
                    if elapsed >= self.wait_duration:
                        rospy.logwarn("Tempo de inspeção finalizado.")
                        self.waiting = False
                        # reseta a variavel 
                        self.marker_reached = 0
                        self.pub_marker_reached.publish(self.marker_reached)
                else:
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

                            if(self.x_sp[self.current_point] == self.end_x and self.y_sp[self.current_point] == self.end_y):
                                self.end_sp = True

                            # se o ponto for o último
                            if(self.end_sp):
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
                                if(self.end_sp):
                                    rospy.logwarn(f"Chegou no destino final")
                                    rospy.logwarn(f"Modo manual ativado")
                                    self.current_point = 0
                                    self.end_sp = False
                                    self.op_mode = 0
                                    self.pub_op_mode.publish(self.op_mode)
                                else:
                                    rospy.logwarn(f"Chegou no marcador {self.current_point+1}")
                                
                                self.waiting = True
                                self.wait_start_time = rospy.Time.now()  # inicia contagem de 10s
                                self.current_point += 1

                                # alcançou o ponto desejado
                                self.marker_reached = 1
                                # publica que chegou no ponto
                                self.pub_marker_reached.publish(self.marker_reached)
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

                            pose_xy = Float32MultiArray()
                            pose_xy.data = [self.x_rot, self.y_rot]
                            self.pub_pose_xy.publish(pose_xy)

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
        position_m2m_node = PositionM2MNode()
        position_m2m_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass
