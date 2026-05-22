#!/usr/bin/env python3

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

# Parameters
w, h = 360, 240 # tamanho da imagem redimensionada largura e altura
fbRange = [6200, 6800] # área minima e máxima do rosto detectado
pid = [0.4, 0.4, 0] # pid
pError = 0 # erro inicial pid
first_image_saved = False

#Para a navegação autônoma para que o drone possa seguir a primeira pessoa que encontrar.
def stop_navigation():
    rospy.sleep(1)
    cancel_navigation_publisher.publish(GoalID())
    rospy.loginfo("Navigation goal canceled.")

#Salva a primeira imagem que contenha uma face
def save_image(img):
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"/home/marcus/catkin_ws/src/tcc_marcus_albano/tello_pkg/images/face_detected_{current_time}.jpg"
    cv2.imwrite(filename, img)
    rospy.loginfo(f"Image saved: {filename}")
    image_path_publisher.publish(filename)

#Detecta a face mais próxima a câmera
def findFace(img):
    global first_image_saved
    faceCascade = cv2.CascadeClassifier('/home/marcus/catkin_ws/src/tcc_marcus_albano/tello_pkg/config/haarcascades/haarcascade_frontalface_default.xml')    # local onde está o arquivo do haarcascade
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                                                         # converte a imagem para escalas de cinza
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

    myFaceListC = []        # armazena o ponto central da face detectada
    myFaceListArea = []     # armazena a area da face detectada

    # cria um retângulo em volta da face e marca o centro com um ponto 
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 2, (0, 255, 0), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
    
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))

        if not first_image_saved:
            save_image(img)
            first_image_saved = True
            #stop_navigation()

        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]

#Define valores de velocidade para o drone seguir o rosto de acordo com o tamanho da área do resto detectado
def trackFace(info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    ySpeed = 0 #velocidade em X

    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    if fbRange[0] < area < fbRange[1]:
        ySpeed = 0
    elif area > fbRange[1]:
        ySpeed = -0.5  
    elif area < fbRange[0] and area != 0:
        ySpeed = 0.5  

    if x == 0:
        speed = 0
        error = 0

    # Criando mensagem Twist
    vel_msg = Twist()
    vel_msg.linear.y = ySpeed
    vel_msg.angular.z = (2*speed) / 100.0  # Convertendo a velocidade calculada para publicar no /tello/cmd_vel

    # Publishing the command
    velocity_publisher.publish(vel_msg)

    return error

def image_callback(msg):
    global pError
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Não foi possível converter a imagem: {e}")
        return

    img = cv2.resize(img, (w, h))
    img, info = findFace(img)
    pError = trackFace(info, w, pid, pError)
    cv2.imshow("Output", img)
    cv2.waitKey(1)

# ROS Node Initialization
rospy.init_node('face_tracker', anonymous=False)

# ROS Publishers and Subscribers
image_subscriber = rospy.Subscriber('/tello/camera/image_raw', Image, image_callback)
velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
image_path_publisher = rospy.Publisher('/file_face_detected', String, queue_size=10)
cancel_navigation_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

bridge = CvBridge()


try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down face tracker.")
    cv2.destroyAllWindows()