#!/usr/bin/env python3
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

# Parameters
w, h = 360, 240 # tamanho da imagem redimensionada largura e altura
fbRange = [6200, 6800] # área minima e máxima do rosto detectado
pid = [0.4, 0.4, 0] # pid
pError = 0 # erro inicial pid
first_image_saved = False

#Para a navegação autônoma para que o drone possa seguir a primeira pessoa que encontrar.

#Salva a primeira imagem que contenha uma face
def save_image(img):
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"/home/marcus/catkin_ws/src/tcc_marcus_albano/tello_pkg/images/face_detected_{current_time}.jpg"
    cv2.imwrite(filename, img)
    rospy.loginfo(f"Image saved: {filename}")

def image_callback(msg):
    
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Não foi possível converter a imagem: {e}")
        return

    cv2.imshow("Output", img)
    cv2.waitKey(1)

# ROS Node Initialization
rospy.init_node('face_tracker', anonymous=False)

# ROS Publishers and Subscribers
image_subscriber = rospy.Subscriber('/tello/camera/image_raw', Image, image_callback)

bridge = CvBridge()

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down face tracker.")
    cv2.destroyAllWindows()