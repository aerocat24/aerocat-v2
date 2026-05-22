import rospy
from tello_driver.msg import TelloStatus
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import tf.transformations

class TestOdomNode:
    
    def __init__(self):
        rospy.init_node('test_odom_node', anonymous=True)


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
        
        self.dt = 0.1 #tempo de amostragem

        self.choice_tag = False

        # Subcribers
        rospy.Subscriber("/tello/status", TelloStatus, self.status_callback)
        rospy.Subscriber('/tello/new_odom', Odometry, self.new_odom_callback)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=False)
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
        self.pub_op_mode = rospy.Publisher('/tello/op_mode', Int32, queue_size=10)

    def status_callback(self, data):
        self.nivel_bat = data.battery_percentage
        self.height_m =  data.height_m
        
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

        rate = rospy.Rate(1) # 1 Hz

        while not rospy.is_shutdown():

            if self.new_odom_data is True:
                rospy.loginfo(f"bat: {self.nivel_bat}, x: {self.distance_x_odom:.2f}, y: {self.distance_y_odom:.2f}, z: {self.distance_z_odom:.2f}, yaw: {self.yaw_odom*57.2958:.2f}")

            rate.sleep()
                
        
if __name__ == '__main__':

    try:
        test_odom_node = TestOdomNode()
        test_odom_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass