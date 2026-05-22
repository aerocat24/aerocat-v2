import rospy
from tello_driver.msg import TelloStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import tf.transformations

class MarkerTrackingNode:
    
    def __init__(self):
        rospy.init_node('marker_tracking_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)

        self.distance_x = None
        self.distance_y = None
        self.distance_z = None
        self.yaw = None
        
        self.op_mode = 0 # 0 manual 1 rastreamento de tag

        # limite bateria: 10%
        self.nivel_bat = None
        
        self.kp_x = 0.5 # ganho proporcional para x
        self.faixa_op_x = 2.0 # faixa de operação do controle [metros]

        self.kp_y = 0.5 # ganho proporcional para y
        self.faixa_op_y = 2.0 # faixa de operação do controle [metros]

        self.kp_z = 1.0 # ganho proporcional para z
        self.faixa_op_z = 1.0 # faixa de operação do controle [metros]

        self.kp_yaw = 1.1152 # ganho proporcional para yaw
        self.faixa_op_yaw = 3.14/4.0 # faixa de operação do controle [rad]

        # ganhos derivativos
        self.kd_x = 0.1
        self.kd_y = 0.1

        # memória dos erros anteriores
        self.error_x_prev = 0.0
        self.error_y_prev = 0.0


        # set points
        self.distance_x_ref = 0.0 # set point para x
        self.distance_y_ref = 0.0 # set point para y
        self.distance_z_ref = 3.0 # set point para z
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

            for detection in data.detections:

                if detection.id[0] not in (0, 1):
                    self.calc_pose(detection.pose.pose.pose.position.x, 
                                   detection.pose.pose.pose.position.y, 
                                   detection.pose.pose.pose.position.z, 
                                   detection.pose.pose.pose.orientation)

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

        # zera todas as velocidades
        vlx = 0 # velocidade linear no eixo x
        vly = 0 # velocidade linear no eixo y
        vlz = 0 # velocidade linear no eixo z
        vaz = 0 # velocidade angular no eixo z
        
        while not rospy.is_shutdown():
            
            #se estiver no modo rastreamento de tag
            if self.op_mode == 1: 
                
                if self.yaw is not None:

                    # error_x, u_x = self.calc_control_action(self.distance_x_ref, self.distance_x, self.kp_x, self.faixa_op_x)
                    # error_y, u_y = self.calc_control_action(self.distance_y_ref, self.distance_y, self.kp_y, self.faixa_op_y)

                    # --- Controle proporcional (já existente)
                    error_x, u_x_p = self.calc_control_action(self.distance_x_ref, self.distance_x, self.kp_x, self.faixa_op_x)
                    error_y, u_y_p = self.calc_control_action(self.distance_y_ref, self.distance_y, self.kp_y, self.faixa_op_y)

                    # --- Controle derivativo (NOVO)
                    dx = (error_x - self.error_x_prev) / self.dt
                    dy = (error_y - self.error_y_prev) / self.dt

                    u_x_d = self.kd_x * dx
                    u_y_d = self.kd_y * dy

                    # salva erros
                    self.error_x_prev = error_x
                    self.error_y_prev = error_y

                    # soma P + D
                    u_x = u_x_p + u_x_d
                    u_y = u_y_p + u_y_d
                    
                    error_yaw, u_yaw = self.calc_control_action(self.yaw_ref, self.yaw, self.kp_yaw, self.faixa_op_yaw)
                    error_z, u_z = self.calc_control_action(self.distance_z_ref, self.distance_z, self.kp_z, self.faixa_op_z)

                    rospy.loginfo(f"bat: {self.nivel_bat},error x: {error_x*100:.2f}, y: {error_y*100:.2f}, z: {error_z*100:.2f}, yaw: {error_yaw*57.2958:.2f}")


                    vlx = -u_x # velocidade linear no eixo x
                    vly = -u_y # velocidade linear no eixo y
                    vlz = u_z # velocidade linear no eixo z
                    vaz = u_yaw # velocidade angular no eixo z

                else:
                    #rospy.loginfo(f"none")
                    vlx = 0 
                    vly = 0 
                    vlz = 0 
                    vaz = 0 

                # envia os comandos de velocidade para o drone
                self.set_cmd_vel(vlx,vly,vlz,vaz)
                
            rate.sleep()


if __name__ == '__main__':

    try:
        marker_tracking_node = MarkerTrackingNode()
        marker_tracking_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass