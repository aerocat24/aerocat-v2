import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf.transformations

class TFModelNode:
    def __init__(self):
        rospy.init_node('tf_model_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.time_elapsed = 0.0
        self.distance_x = None
        self.distance_y = None
        self.distance_z = None
        self.yaw_rad = None
        self.yaw = None

    def callback(self, data):

        if data.detections:  # Verifica se há alguma tag detectada
            self.distance_x = - data.detections[0].pose.pose.pose.position.x # metros
            self.distance_y = - data.detections[0].pose.pose.pose.position.y # metros
            self.distance_z = data.detections[0].pose.pose.pose.position.z   # metros

            # obtem o quaternion da orientação do apriltag
            quat = data.detections[0].pose.pose.pose.orientation
            quaternion = [quat.x, quat.y, quat.z, quat.w]
            
            # converte o quaternion para ângulos de euler (roll, pitch, yaw)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw_rad = euler[2]  # O terceiro valor é o ângulo yaw (em rad)

            # Converte para graus
            self.yaw = self.yaw_rad * (180.0 / 3.14159265359)

            print(f"X:{self.distance_x*100:.2f}, Y:{self.distance_y*100:.2f}, Z:{self.distance_z*100:.2f},YAW:{self.yaw :.2f}")

        else:
            self.distance_x = None
            self.distance_y = None
            self.distance_z = None
            self.yaw = None

    def timer_callback(self, event):
        # if self.distance_x is not None:
        #    print(f"{self.time_elapsed:.2f},{self.distance_x:.2f}")
        # else:
        #    print(f"{self.time_elapsed:.2f},None")
        self.time_elapsed += 0.1

if __name__ == '__main__':
    try:
        node = TFModelNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass