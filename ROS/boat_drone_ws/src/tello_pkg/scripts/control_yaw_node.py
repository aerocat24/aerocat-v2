import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tf.transformations

class ControlYawNode:
    
    def __init__(self):
        rospy.init_node('control_yaw_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

        self.yaw = None
        
        self.kp = 1.1152 # ganho proporcional
        self.faixa_op = 3.14/4.0 # faixa de operação do controle [rad]

        self.yaw_ref = 0.0 # set point [rad]
        self.dt = 0.1 # tempo de amostragem

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)

        rospy.on_shutdown(self.shutdown_hook)  

    def shutdown_hook(self):
        self.set_cmd_vel(0,0,0,0)
    
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

    def callback(self, data):
        if data.detections:  # Verifica se há alguma tag detectada
            # obtem o quaternion da orientação do apriltag
            quat = data.detections[0].pose.pose.pose.orientation
            quaternion = [quat.x, quat.y, quat.z, quat.w]
            
            # converte o quaternion para ângulos de euler (roll, pitch, yaw)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw_rad = euler[2]  # O terceiro valor é o ângulo yaw (em rad)

            # Converte para graus
            # self.yaw = yaw_rad * (180.0 / 3.14159265359)
            self.yaw = yaw_rad 
        else:
            self.yaw = None

    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s
        u = 0
        # zera todas as velocidades
        vlx = 0 # velocidade linear no eixo x
        vly = 0 # velocidade linear no eixo y
        vlz = 0 # velocidade linear no eixo z
        vaz = 0 # velocidade angular no eixo z
        
        while not rospy.is_shutdown():

            if self.yaw is not None:

                error = self.yaw_ref - self.yaw

                if error > self.faixa_op:
                    # velocidade máxima 
                    u = 1.0
                else:
                    # calcula a ação
                    u = self.kp*error
                
                # saturador
                if u > 1.0:
                    u = 1.0
                elif u < -1.0:
                    u = -1.0
            else:
                # caso nao tenha uma leitura da tag válida
                u = 0
            
            #velocidade no eixo yaw
            vaz = u

            self.set_cmd_vel(vlx,vly,vlz,vaz)
            
            if self.yaw is not None:
                rospy.loginfo(f"yaw: {self.yaw* (180.0 / 3.14159265359):.2f}, yaw_ref: {self.yaw_ref:.2f}, acao: {u:.2f}")
            else:
                rospy.loginfo(f"d: none, d_ref: {self.yaw_ref:.2f}, acao: {u:.2f}")

            rate.sleep()


if __name__ == '__main__':

    try:
        control_yaw_node = ControlYawNode()
        control_yaw_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass