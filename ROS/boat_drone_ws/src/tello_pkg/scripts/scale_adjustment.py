#!/usr/bin/env python3
"""
Utilizar esse script após ter o valor da escala definida.
"""
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np


def scale_point_cloud(point_cloud, scale_factor):
    """
    Escala a nuvem de pontos pelo fator de escala fornecido.
    """
    #usar esses indices apenas no mapa real.
    #indices_to_zero_z=[208, 234, 235, 245, 496, 525, 617, 1046, 1413, 1789, 1876, 2062, 2548, 2695, 2696, 2700, 2704, 2707, 2812, 2920, 2924, 3505, 3783, 4184, 4361, 4362, 4720, 5418, 5445, 5450, 5451, 5462, 5481, 5524, 5743, 5858, 5980, 5990, 6089, 6096, 6279, 6448, 6550, 7045, 7084, 7454, 7753, 7943, 8052, 8423, 8557, 8658, 8685, 9942, 9943, 9945, 9969, 9975, 10144, 10178, 10306, 10312, 10313, 10631, 11931, 11957, 12150, 12158, 12159, 12238, 12374, 13348] #setar eixo z para remover pontos do mapa.
    points = np.array(list(pc2.read_points(point_cloud,field_names=("x", "y", "z"), skip_nans=True)))
    #points[indices_to_zero_z, 2] = 0
    points[:, :3] *= scale_factor
    modified_points = [((x/1.5)+1.3, (y*1.25), z+ 1.2) for x, y, z in points] #usar com gazebo
    #modified_points = [(x, y, z+ 1.2) for x, y, z in points] #usar no mapa real.

    
    scaled_point_cloud = pc2.create_cloud(point_cloud.header, point_cloud.fields, modified_points)
    return scaled_point_cloud
    

def callback(point_cloud):
    """
    Callback para processar a nuvem de pontos recebida.
    """
    # Colocar o valor da escala calculada 
    scale_factor = 6.498548086326428 #rosbag = teste2.bag
    #scale_factor = 6.018648579486336#5.564094954455052

    #scale_factor = 3.8455560957865367 #rosbag = subset.bag
    # scale_factor = 8.384939138795436 #rosbag = subset2.bag

    # Escalar a nuvem de pontos
    scaled_cloud = scale_point_cloud(point_cloud, scale_factor)
    pub.publish(scaled_cloud)

if __name__ == '__main__':
    rospy.init_node('scale_point_cloud_node', anonymous=True)
    rospy.Subscriber('/orb_slam3/all_points', PointCloud2, callback)
    pub = rospy.Publisher('/scaled_point_cloud', PointCloud2, queue_size=10)
    rospy.spin()
