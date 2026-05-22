import rospy
from tello_driver.msg import TelloStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import tf.transformations

class PrecisionLadingNode:
    
    def __init__(self):
        rospy.init_node('precision_lading_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)

        self.op_mode = 0 # 0 manual 2 pouso de precisão

        self.distance_x = None
        self.distance_y = None
        self.distance_z = None
        self.yaw = None
        
        # limite bateria: 10%
        self.nivel_bat = None
        
        self.kp_x = 0.5186  # ganho proporcional para x
        self.faixa_op_x = 1.0 # faixa de operação do controle [metros]
        self.faixa_erro_land_x = 0.05 # [metros]
        self.faixa_erro_centro_x = 0.5 # [metros]

        self.kp_y = 0.5510 # ganho proporcional para y
        self.faixa_op_y = 1.0 # faixa de operação do controle [metros]
        self.faixa_erro_land_y = 0.05 # [metros]
        self.faixa_erro_centro_y = 0.5 # [metros]

        self.kp_z = 0.7132 # ganho proporcional para z
        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]
        self.faixa_erro_land_z = 0.15 # [metros]
        # self.faixa_erro_centro_z = 0.5 # [metros] # not used

        self.kp_yaw = 1.1152 # ganho proporcional para yaw
        self.faixa_op_yaw = 3.14/4.0 # faixa de operação do controle [rad]
        self.faixa_erro_land_yaw = 5 * (3.14159265359 / 180.0) # [rad]
        # self.faixa_erro_centro_yaw = 5 * (3.14159265359 / 180.0) # [rad] # not used

        # set points
        self.distance_x_ref = -0.05 # set point para x
        self.distance_y_ref = 0.0 # set point para y
        self.distance_z_ref = 0.35 # set point para z
        self.yaw_ref = 90 * (3.14159265359 / 180.0) # set point para yaw

        self.dt = 0.1 #tempo de amostragem

        self.choice_tag = False

        # Subcribers
        rospy.Subscriber("/tello/status", TelloStatus, self.status_callback)
        rospy.Subscriber("/tello/op_mode", Int32, self.op_mode_callback)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)

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
    
    def calc_pose(self, dist_x, dist_y, dist_z, quat):
        self.distance_x = dist_x   # [metros]
        self.distance_y = dist_y   # [metros]
        self.distance_z = dist_z   # [metros]

        # obtem o quaternion da orientação do apriltag
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        
        # converte o quaternion para ângulos de euler (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_rad = euler[2]  # O terceiro valor é o ângulo yaw (em rad)

        self.yaw = yaw_rad 

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

    def tag_detection_callback(self, data):

        if data.detections:

            self.choice_tag = False

            for detection in data.detections:

                # print(f"id: {detection.id} z: {detection.pose.pose.pose.position.z}")
                # prioridade de detecção da tag maior quando a altitude for maior que 1.0 metro
                if detection.pose.pose.pose.position.z >= 1.5 and detection.id[0] == 0: 
                    self.choice_tag = True
                    self.calc_pose(detection.pose.pose.pose.position.x, 
                                   detection.pose.pose.pose.position.y, 
                                   detection.pose.pose.pose.position.z, 
                                   detection.pose.pose.pose.orientation)

                # prioridade de detecção na tag menor quando a altura for menor que 1.0m
                elif detection.pose.pose.pose.position.z < 1.5 and detection.id[0] == 1: 
                    self.choice_tag = True
                    self.calc_pose(detection.pose.pose.pose.position.x, 
                                   detection.pose.pose.pose.position.y, 
                                   detection.pose.pose.pose.position.z, 
                                   detection.pose.pose.pose.orientation)
                    # print(f"id: {detection.id[0]}, X:{self.distance_x*100:.2f}, Y:{self.distance_y*100:.2f}, Z:{self.distance_z*100:.2f},YAW:{self.yaw*(180.0 / 3.14159265359) :.2f}")
                    
                    
            # caso não tenha detectado a tag prioritária, utiliza a primeira tag detectada
            if self.choice_tag is False:

                # apenas vai utilizar as tags 0 ou 1 para o pouso de precisão
                if detection.id[0] == 0 or detection.id[0] == 1:
                    self.calc_pose(data.detections[0].pose.pose.pose.position.x, 
                                data.detections[0].pose.pose.pose.position.y, 
                                data.detections[0].pose.pose.pose.position.z, 
                                data.detections[0].pose.pose.pose.orientation)
                else:
                    self.distance_x = None
                    self.distance_y = None
                    self.distance_z = None
                    self.yaw = None
        else:
            self.distance_x = None
            self.distance_y = None
            self.distance_z = None
            self.yaw = None

    def op_mode_callback(self, msg):
        self.op_mode = msg.data

    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s
        u_x = 0
        u_y = 0
        u_z = 0
        u_yaw = 0

        landing = False

        cont_land = 0

        # zera todas as velocidades
        vlx = 0 # velocidade linear no eixo x
        vly = 0 # velocidade linear no eixo y
        vlz = 0 # velocidade linear no eixo z
        vaz = 0 # velocidade angular no eixo z
        
        while not rospy.is_shutdown():

            # se estiver no modo pouso de precisão 
            if self.op_mode == 2:

                if self.yaw is not None:

                    if self.distance_z >= 1.5:
                        self.kp_x = 0.5186  # ganho proporcional para x
                        self.kp_y = 0.5510 # ganho proporcional para y
                        self.kp_z = 0.7132 # ganho proporcional para z
                        self.faixa_erro_centro_x = 0.5 # [metros]
                        self.faixa_erro_centro_y = 0.5 # [metros]
                        self.faixa_op_x = 1.0 # faixa de operação do controle [metros]
                        self.faixa_op_y = 1.0 # faixa de operação do controle [metros]
                        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]
        
                    elif self.distance_z < 1.5 and self.distance_z >= 0.75:
                        self.kp_x = 0.8149  # ganho proporcional para x
                        self.kp_y = 0.7577 # ganho proporcional para y
                        self.faixa_erro_centro_x = 0.25 # [metros]
                        self.faixa_erro_centro_y = 0.25 # [metros]
                        self.faixa_op_x = 1.0 # faixa de operação do controle [metros]
                        self.faixa_op_y = 1.0 # faixa de operação do controle [metros]
                        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]
                    else:
                        self.kp_x = 1.9015  # ganho proporcional para x
                        self.kp_y = 2.0205 # ganho proporcional para y
                        self.kp_z = 0.9875 # ganho proporcional para z
                        self.faixa_erro_centro_x = 0.15 # [metros]
                        self.faixa_erro_centro_y = 0.15 # [metros]
                        self.faixa_op_x = 0.5 # faixa de operação do controle [metros]
                        self.faixa_op_y = 0.5 # faixa de operação do controle [metros]
                        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]

                    error_x, u_x = self.calc_control_action(self.distance_x_ref, self.distance_x, self.kp_x, self.faixa_op_x)
                    error_y, u_y = self.calc_control_action(self.distance_y_ref, self.distance_y, self.kp_y, self.faixa_op_y)
                    error_yaw, u_yaw = self.calc_control_action(self.yaw_ref, self.yaw, self.kp_yaw, self.faixa_op_yaw)

                    
                    if abs(error_x) < self.faixa_erro_centro_x and abs(error_y) < self.faixa_erro_centro_y: 
                        error_z, u_z = self.calc_control_action(self.distance_z_ref, self.distance_z, self.kp_z, self.faixa_op_z)
                        rospy.loginfo(f"bat: {self.nivel_bat},error x: {error_x*100:.2f}, y: {error_y*100:.2f}, z: {error_z*100:.2f}, yaw: {error_yaw*57.2958:.2f}")
                    else:
                        u_z = 0
                        error_z = None
                        rospy.loginfo(f"bat: {self.nivel_bat},error x: {error_x*100:.2f}, y: {error_y*100:.2f}, z: None, yaw: {error_yaw*57.2958:.2f}")


                    vlx = -u_x # velocidade linear no eixo x
                    vly = -u_y # velocidade linear no eixo y
                    vlz = u_z # velocidade linear no eixo z
                    vaz = u_yaw # velocidade angular no eixo z

                    if error_z is not None:
                        if abs(error_z) < self.faixa_erro_land_z and abs(error_x) < self.faixa_erro_land_x and abs(error_y) < self.faixa_erro_land_y and abs(error_yaw) < self.faixa_erro_land_yaw:
                            if landing is False:
                                landing = True
                                cont_land = 0

                    if landing is True:
                        if cont_land == 0:
                            vlx = 0
                            vly = 0
                            vlz = 0
                            vaz = 0
                            self.set_cmd_vel(vlx,vly,vlz,vaz)
                            rospy.loginfo(f"Landing...")
                            self.land()
                            cont_land += 1
                        elif cont_land >= 50:
                            landing = False
                        else:
                            cont_land += 1
                    else:    
                        self.set_cmd_vel(vlx,vly,vlz,vaz) 

                else:
                    #rospy.loginfo(f"none")
                    vlx = 0 
                    vly = 0 
                    vlz = 0 
                    vaz = 0 

                # envia os comandos de velocidade para o drone
                self.set_cmd_vel(vlx,vly,vlz,vaz)
            else:
                self.distance_x = None
                self.distance_y = None
                self.distance_z = None
                self.yaw = None
                
            rate.sleep()


if __name__ == '__main__':

    try:
        precision_lading_node = PrecisionLadingNode()
        precision_lading_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass