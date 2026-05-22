import rospy
from tello_driver.msg import TelloStatus
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
import math

class MarkerIdentificationNode:
    
    def __init__(self):

        rospy.init_node('marker_identification_node', anonymous=True)

        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)

        self.new_odom = None
        self.detections = None


        self.dt = 0.1 #tempo de amostragem

        # listas para armazenar os id das tags identificadas 
        self.markers_identified = []

        self.cont = 0

        # Subcribers
        rospy.Subscriber('/tello/new_odom', Odometry, self.new_odom_callback)


        # Publishers
        self.pub_pos_xy_marker = rospy.Publisher('/pos_xy_marker', Float32MultiArray, queue_size=10)


    def new_odom_callback(self, msg):
        
        self.new_odom = msg

        # obtem as posições x, y, z da odometria, porém inverte os eixos x e -y para ficar na mesma referência da velocidade do drone
        self.distance_x_odom = -self.new_odom.pose.pose.position.y
        self.distance_y_odom = self.new_odom.pose.pose.position.x


    def tag_detection_callback(self, data):

        self.detections = data

    def run(self):

        rate = rospy.Rate(10) #10Hz -> 0.1s 

        while not rospy.is_shutdown():

            if self.detections is not None:
                
                if self.detections.detections:

                    for detection in self.detections.detections:

                        # se identificar as tags diferentes de 0 e 1
                        if detection.id[0] not in (0, 1):
                            # se a tag ainda não foi identificada
                            if detection.id[0] not in self.markers_identified:
                                    self.markers_identified.append(detection.id[0])
                                    rospy.logwarn(f"Tag {detection.id[0]} identificada.")
                                    #obtem as posições relativas da tag em relação ao drone
                                    self.distance_x_relative = detection.pose.pose.pose.position.x
                                    self.distance_y_relative = detection.pose.pose.pose.position.y #inverte o eixo y
                                    # calcula a posição da tag no ambiente
                                    pos_x_marker = self.distance_x_odom + self.distance_x_relative
                                    pos_y_marker = self.distance_y_odom + self.distance_y_relative

                                    # envia a posição xy do marcador
                                    pos_xy_marker = Float32MultiArray()
                                    pos_xy_marker.data = [detection.id[0], pos_x_marker, pos_y_marker]
                                    self.pub_pos_xy_marker.publish(pos_xy_marker)

                                    rospy.loginfo(f"posicao relativa: ({self.distance_x_relative:.2f}, {self.distance_y_relative:.2f}) [m]")
                                    rospy.loginfo(f"posicao odom: ({self.distance_x_odom:.2f}, {self.distance_y_odom:.2f}) [m]")
                                    rospy.loginfo(f"posicao marker: ({pos_x_marker:.2f}, {pos_y_marker:.2f}) [m]")

            rate.sleep()

if __name__ == '__main__':

    try:
        marker_identification_node = MarkerIdentificationNode()
        marker_identification_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass