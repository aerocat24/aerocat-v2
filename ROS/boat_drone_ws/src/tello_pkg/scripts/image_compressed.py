#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Inicializa o CvBridge para conversão entre ROS Image e OpenCV
bridge = CvBridge()

def image_raw_callback(msg):
    try:
        # Converte a mensagem image_raw (ROS Image) para OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Comprime a imagem usando OpenCV
        _, compressed_image = cv2.imencode('.jpg', cv_image)

        # Prepara a mensagem CompressedImage
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()

        # Publica a imagem comprimida
        compressed_image_pub.publish(compressed_msg)

    except CvBridgeError as e:
        rospy.logerr("Erro ao converter image_raw: %s" % e)

if __name__ == "__main__":
    # Inicializa o nó ROS
    rospy.init_node('image_compression_node', anonymous=False)

    # Publisher para image_compressed
    compressed_image_pub = rospy.Publisher('/tello/camera/image_raw/compressed', CompressedImage, queue_size=1)

    # Listener para image_raw
    rospy.Subscriber('/tello/camera/image_raw', Image, image_raw_callback)

    rospy.spin()
