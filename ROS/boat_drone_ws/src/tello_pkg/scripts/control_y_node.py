import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ControlYNode:
    
    def __init__(self):
        rospy.init_node('control_y_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        
        self.distance = None
        
        self.kp = 0.5510 # ganho proporcional
        self.faixa_op = 1.0 # faixa de operação do controle [metros]
        
        self.distance_ref = 0.0 # set point
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
            self.distance = data.detections[0].pose.pose.pose.position.y
        else:
            self.distance = None

    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s
        u = 0
        # zera todas as velocidades
        vlx = 0 # velocidade linear no eixo x
        vly = 0 # velocidade linear no eixo y
        vlz = 0 # velocidade linear no eixo z
        vaz = 0 # velocidade angular no eixo z
        
        while not rospy.is_shutdown():

            if self.distance is not None:

                error = self.distance_ref - self.distance

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
            
            # velocidade no eixo z
            vly = -u

            self.set_cmd_vel(vlx,vly,vlz,vaz)

            if self.distance is not None:
                rospy.loginfo(f"d: {self.distance*100:.2f}, d_ref: {self.distance_ref*100:.2f}, acao: {u:.2f}")
            else:
                rospy.loginfo(f"d: none, d_ref: {self.distance_ref*100:.2f}, acao: {u:.2f}")
                

            rate.sleep()


if __name__ == '__main__':

    try:
        control_y_node = ControlYNode()
        control_y_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass