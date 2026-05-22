import rospy
from tello_driver.msg import TelloStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import tf.transformations

class ScanNode:
    
    def __init__(self):
        rospy.init_node('scan_node', anonymous=True)

        self.op_mode = 0 # 0 manual 1 navegação autônoma 

        # valores recebidos da odometria
        self.distance_x_odom = None
        self.distance_y_odom = None
        self.distance_z_odom = None
        self.yaw_odom = None

        self.new_odom = None
        self.new_odom_data = False
        
        # limite bateria: 10%
        self.nivel_bat = None
        self.height_m = None
        
        self.kp_x = 0.814/2.0 # ganho proporcional para x
        self.faixa_op_x = 2.0 # faixa de operação do controle [metros]

        self.kp_y = 0.7577/2.0  # ganho proporcional para y
        self.faixa_op_y = 2.0 # faixa de operação do controle [metros]

        self.kp_z = 0.7132 # ganho proporcional para z
        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]

        self.kp_yaw = 1.1152 # ganho proporcional para yaw
        self.faixa_op_yaw = 3.14/4.0 # faixa de operação do controle [rad] # 45 graus

        self.waypoint_index = 0 # índice do waypoint atual

        # x, y, z, yaw
        # teste ao lado da piscina
        # self.waypoints = [
        #     [0.0, 0.0, 0.0, 0.0], # origem
        #     [0.0, 1.5, 3.0, 0.0],
        #     [0.0, 12.5, 3.0, 0.0],
        #     [4.0, 12.5, 3.0, 0.0],
        #     [5.0, 15.0, 3.0, 0.0],
        # ]

        self.waypoints = [
            [0.0, 0.0, 0.0, 0.0],   # origem
            [1.5, 1.0, 3.0, 0.0],   # wp 1
            [9.0, 1.0, 3.0, 0.0],  # wp 2
            [9.0, 3.5, 3.0, 0.0],  # wp 3
            [3.0, 3.5, 3.0, 0.0],   # wp 4
            [3.0, 6.0, 3.0, 0.0],   # wp 5
            [9.0, 6.0, 3.0, 0.0],  # wp 6
            [9.0, 8.5, 3.0, 0.0],  # wp 7
            [3.0, 8.5, 3.0, 0.0],   # wp 8
            [3.0, 11.0, 3.0, 0.0],   # wp 9
            [9.0, 11.0, 3.0, 0.0],  # wp 10
            [9.0, 13.5, 3.0, 0.0],  # wp 11
            [3.0, 13.5, 3.0, 0.0],  # wp 12
            [3.0, 16.0, 3.0, 0.0],  # wp 13
            [9.0, 16.0, 3.0, 0.0],  # wp 14
            [9.0, 18.5, 3.0, 0.0],  # wp 15
            [3.0, 18.5, 3.0, 0.0],  # wp 16
            [3.0, 21.5, 3.0, 0.0],  # wp 17
            [9.0, 21.5, 3.0, 0.0],  # wp 18
            [9.0, 24.0, 3.0, 0.0],  # wp 19
            [3.0, 24.0, 3.0, 0.0],  # wp 20
            [16, 27.0, 3.0, 0.0],  # wp 21
        ]

        # self.waypoints = [
        #     [0.0, 0.0, 0.0, 0.0],   # origem
        #     [1.5, 1.0, 1.5, 0.0],   # wp 1
        #     [11.5, 1.0, 1.5, 0.0],  # wp 2
        #     [11.5, 3.0, 3.0, 0.0],  # wp 3
        #     [1.5, 3.0, 3.0, 0.0],   # wp 4
        #     [1.5, 5.0, 3.0, 0.0],   # wp 5
        #     [11.5, 5.0, 3.0, 0.0],  # wp 6
        #     [11.5, 7.0, 3.0, 0.0],  # wp 7
        #     [1.5, 7.0, 3.0, 0.0],   # wp 8
        #     [1.5, 9.0, 3.0, 0.0],   # wp 9
        #     [11.5, 9.0, 3.0, 0.0],  # wp 10
        #     [11.5, 11.0, 3.0, 0.0], # wp 11
        #     [1.5, 11.0, 3.0, 0.0],  # wp 12
        #     [1.5, 13.0, 3.0, 0.0],  # wp 13
        #     [11.5, 13.0, 3.0, 0.0], # wp 14
        #     [11.5, 15.0, 3.0, 0.0], # wp 15
        #     [1.5, 15.0, 3.0, 0.0],  # wp 16
        #     [1.5, 17.0, 3.0, 0.0],  # wp 17
        #     [11.5, 17.0, 3.0, 0.0], # wp 18
        #     [11.5, 19.0, 3.0, 0.0], # wp 19
        #     [1.5, 19.0, 3.0, 0.0],  # wp 20
        #     [1.5, 21.0, 3.0, 0.0],  # wp 21
        #     [11.5, 21.0, 3.0, 0.0], # wp 22
        #     [12.5, 25.0, 3.0, 0.0], # wp 23
        #     # [1.5, 23.0, 1.5, 0.0],  # wp 24
        # ]

        # x, y, z, yaw
        # self.waypoints = [
        #     [0.0, 0.0, 0.0, 0.0],   # origem
        #     [5.0, 3.0, 3.0, 0.0],   # wp 1
        #     [-1.0, 5.0, 3.0, 0.0]   # wp 1
        # ]

        # # x, y, z, yaw
        # self.waypoints = [
        #     [0.0, 0.0, 0.0, 0.0],   # origem
        #     [1.5, 1.0, 3.0, 0.0],   # wp 1
        #     [11.5, 1.0, 3.0, 0.0],  # wp 2
        #     [11.5, 3.0, 3.0, 0.0],  # wp 3
        #     [1.5, 3.0, 3.0, 0.0],   # wp 4
        #     [1.5, 5.0, 3.0, 0.0],   # wp 5
        #     [11.5, 5.0, 3.0, 0.0],  # wp 6
        #     [11.5, 7.0, 3.0, 0.0],  # wp 7
        #     [1.5, 7.0, 3.0, 0.0],   # wp 8
        #     [1.5, 9.0, 3.0, 0.0],   # wp 9
        #     [11.5, 9.0, 3.0, 0.0],  # wp 10
        #     [11.5, 11.0, 3.0, 0.0], # wp 11
        #     [1.5, 11.0, 3.0, 0.0],  # wp 12
        #     [1.5, 13.0, 3.0, 0.0],  # wp 13
        #     [11.5, 13.0, 3.0, 0.0], # wp 14
        #     [11.5, 15.0, 3.0, 0.0], # wp 15
        #     [1.5, 15.0, 3.0, 0.0],  # wp 16
        #     [1.5, 17.0, 3.0, 0.0],  # wp 17
        #     [11.5, 17.0, 3.0, 0.0], # wp 18
        #     [11.5, 19.0, 3.0, 0.0], # wp 19
        #     [1.5, 19.0, 3.0, 0.0],  # wp 20
        #     [1.5, 21.0, 3.0, 0.0],  # wp 21
        #     [11.5, 21.0, 3.0, 0.0], # wp 22
        #     [11.5, 23.0, 3.0, 0.0], # wp 23
        #     [1.5, 23.0, 3.0, 0.0],  # wp 24
        #     [1.5, 25.5, 3.0, 0.0],  # wp 25
        # ]

        self.dt = 0.1 #tempo de amostragem

        self.choice_tag = False

        # Subcribers
        rospy.Subscriber("/tello/status", TelloStatus, self.status_callback)
        rospy.Subscriber("/tello/op_mode", Int32, self.op_mode_callback)
        rospy.Subscriber('/tello/new_odom', Odometry, self.new_odom_callback)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
        self.pub_op_mode = rospy.Publisher('/tello/op_mode', Int32, queue_size=10)

        rospy.on_shutdown(self.shutdown_hook)  

    def shutdown_hook(self):
        self.set_cmd_vel(0,0,0,0)

    
    def calc_control_action(self, var_ref, var, kp, faixa_op):
        """
        Calcular ação de controle proporcional

        Parâmetros:
        var_ref (int ou float): variável de referência

        var (int ou float): variável a ser controlada.

        kp (int ou float): ganho proporcional

        faixa_op (int ou float): faixa de operação do controle.

        Retorna:
        int ou float: ação proporcional.
        """

        error = var_ref - var

        if error > faixa_op:
            # velocidade máxima 
            u = 1.0
        else:
            #calcula a ação 
            u = kp*error

            # saturador
            if u > 1.0:
                u = 1.0
            elif u < -1.0:
                u = -1.0
        
        return error, u 

    # função para enviar comandos de velocidade para o drone
    def set_cmd_vel(self,vlx,vly,vlz,vaz):
        vel_msg = Twist()
        vel_msg.linear.x = float(vlx)
        vel_msg.linear.y = float(vly)
        vel_msg.linear.z = float(vlz)
        vel_msg.angular.z = float(vaz)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        self.pub_cmd_vel.publish(vel_msg)

    # função para pousar o drone
    def land(self):
        self.pub_land.publish()
    
    # função para decolar o drone
    def takeoff(self):
        self.pub_takeoff.publish()

    def status_callback(self, data):
        self.nivel_bat = data.battery_percentage
        self.height_m =  data.height_m

    def op_mode_callback(self, msg):
        self.op_mode = msg.data
    
    def new_odom_callback(self, msg):
        self.new_odom = msg
        # obtem o quaternion da orientação do apriltag
        quaternion = [self.new_odom.pose.pose.orientation.x, 
                      self.new_odom.pose.pose.orientation.y, 
                      self.new_odom.pose.pose.orientation.z, 
                      self.new_odom.pose.pose.orientation.w]
        
        # converte o quaternion para ângulos de euler (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_odom = euler[2]  # O terceiro valor é o ângulo yaw (em rad)

        # obtem as posições x, y, z da odometria, porém inverte os eixos x e -y para ficar na mesma referência da velocidade do drone
        self.distance_x_odom = -self.new_odom.pose.pose.position.y
        self.distance_y_odom = self.new_odom.pose.pose.position.x
        self.distance_z_odom = self.new_odom.pose.pose.position.z #esta acumulando muito erro
        #self.distance_z_odom = self.height_m

        self.new_odom_data = True


    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s
        u_x = 0
        u_y = 0
        u_z = 0
        u_yaw = 0

        # zera todas as velocidades
        vlx = 0 # velocidade linear no eixo x
        vly = 0 # velocidade linear no eixo y
        vlz = 0 # velocidade linear no eixo z
        vaz = 0 # velocidade angular no eixo z

        # começa com o início do waypoint 1
        self.waypoint_index = self.waypoint_index + 1
        
        while not rospy.is_shutdown():

            # if self.new_odom_data is True:
            #     rospy.loginfo(f"bat: {self.nivel_bat}, x: {self.distance_x_odom:.2f}, y: {self.distance_y_odom:.2f}, z: {self.distance_z_odom:.2f}, yaw: {self.yaw_odom*57.2958:.2f}")

            # se estiver no modo navegação autônoma
            if self.op_mode == 1: 

                if self.new_odom_data is True:

                    # obtem o waypoint atual
                    waypoint = self.waypoints[self.waypoint_index]

                    # obtem as referências do waypoint atual
                    self.distance_x_ref = waypoint[0]
                    self.distance_y_ref = waypoint[1]
                    self.distance_z_ref = waypoint[2]
                    self.yaw_ref = waypoint[3]

                    # calcula as ações de controle para cada eixo
                    error_x, u_x = self.calc_control_action(self.distance_x_ref, self.distance_x_odom, self.kp_x, self.faixa_op_x)
                    error_y, u_y = self.calc_control_action(self.distance_y_ref, self.distance_y_odom, self.kp_y, self.faixa_op_y)
                    error_z, u_z = self.calc_control_action(self.distance_z_ref, self.height_m, self.kp_z, self.faixa_op_z)
                    error_yaw, u_yaw = self.calc_control_action(self.yaw_ref, self.yaw_odom, self.kp_yaw, self.faixa_op_yaw)

                    rospy.loginfo(f"bat: {self.nivel_bat},wp_index: {self.waypoint_index}, error x: {error_x:.2f}, y: {error_y:.2f}, z: {error_z:.2f}, yaw: {error_yaw*57.2958:.2f}")
                    
                    vlx = u_x # velocidade linear no eixo x
                    vly = u_y # velocidade linear no eixo y
                    vlz = u_z # velocidade linear no eixo z
                    vaz = u_yaw # velocidade angular no eixo z

                    # verifica se chegou perto o suficiente do waypoint atual
                    dist_to_wp = (error_x**2 + error_y**2)**0.5

                    if dist_to_wp < 1.0 and abs(error_z) < 1.25 and abs(error_yaw) < (3.14/18.0): # 10 graus

                        # se chegou, passa para o próximo waypoint
                        if self.waypoint_index < len(self.waypoints) - 1:
                            self.waypoint_index += 1
                            rospy.logwarn(f"Reached waypoint {self.waypoint_index}, moving to next waypoint.")
                        else:
                            rospy.logwarn("All waypoints reached. Hovering in place.")

                            vlx = 0
                            vly = 0
                            vlz = 0
                            vaz = 0

                            rospy.logwarn("Changing to precision landing mode.")
                            # muda para o modo de pouso de precisão
                            self.op_mode = 2
                            self.pub_op_mode.publish(self.op_mode)

                    # envia os comandos de velocidade para o drone
                    self.set_cmd_vel(vlx,vly,vlz,vaz)
                    
                    # reseta a variável para aguardar uma nova odometria
                    self.new_odom_data = False

                else:
                    vlx = 0 
                    vly = 0 
                    vlz = 0 
                    vaz = 0 

                # envia os comandos de velocidade para o drone
                self.set_cmd_vel(vlx,vly,vlz,vaz)

            rate.sleep()
                
        
if __name__ == '__main__':

    try:
        scan_node = ScanNode()
        scan_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass