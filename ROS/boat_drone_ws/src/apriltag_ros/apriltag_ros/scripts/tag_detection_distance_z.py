#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def callback(data):
    if data.detections:  # Verifica se há alguma tag detectada
        for detection in data.detections:
            # Acessa a posição z da tag no espaço do frame de referência
            z_distance = detection.pose.pose.pose.position.z * 100
            tag_id = detection.id[0]  # ID da tag
            rospy.loginfo(f"id da tag: {tag_id}, distância Z: {z_distance} cm")
            #print(f"Distância Z: {z_distance} cm")
    else:
        rospy.loginfo("Nenhuma tag detectada.")
        #print(f"Nenhuma tag detectada.")

def april_tag_distance_listener():
    rospy.init_node('april_tag_distance_listener', anonymous=True)

    # Substitua '/tag_detections' pelo tópico real de deteções de AprilTags no seu sistema
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        april_tag_distance_listener()
    except rospy.ROSInterruptException:
        pass