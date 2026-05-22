#!/usr/bin/env python3
"""
Utilizar esse script para calcular a escala que será utilizada.
"""
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

def calculate_scale_factor(point1, point2, real_distance):
    """
    Calcula o fator de escala com base na distância real entre dois pontos e a distância na nuvem de pontos.
    """
    distance_in_cloud = np.linalg.norm(point1 - point2)
    scale_factor = real_distance / distance_in_cloud
    return scale_factor

def scale_point_cloud(point_cloud, scale_factor):
    """
    Escala a nuvem de pontos pelo fator de escala fornecido.
    """
    points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))
    points[:, :3] *= scale_factor
    scaled_point_cloud = pc2.create_cloud(point_cloud.header, point_cloud.fields, points)
    return scaled_point_cloud

def callback(point_cloud):
    """
    Callback para processar a nuvem de pontos recebida.
    """
    # Identificar os dois pontos conhecidos (substitua pelos índices ou coordenadas corretas)
    known_point1_index = 7983#7650  # Índice do primeiro ponto na nuvem de pontos
    known_point2_index = 10929#13010  # Índice do segundo ponto na nuvem de pontos

    real_distance = 8  # Distância real entre os dois pontos no mundo real (em metros)

    points = np.array(list(pc2.read_points(point_cloud, skip_nans=True)))

    # Verificar se os índices dos pontos são válidos
    if known_point1_index >= len(points) or known_point2_index >= len(points):
        rospy.logwarn("Índices dos pontos conhecidos são inválidos.")
        return

    # Encontrar as coordenadas dos pontos na nuvem de pontos
    point1 = points[known_point1_index][:3]
    point2 = points[known_point2_index][:3]

    # Calcular o fator de escala
    scale_factor = calculate_scale_factor(point1, point2, real_distance)
    rospy.loginfo(f'Fator de escala calculado: {scale_factor}')

    # Escalar a nuvem de pontos
    scaled_cloud = scale_point_cloud(point_cloud, scale_factor)
    pub.publish(scaled_cloud)

if __name__ == '__main__':
    rospy.init_node('scale_point_cloud_node', anonymous=True)
    rospy.Subscriber('/orb_slam3/all_points', PointCloud2, callback)
    pub = rospy.Publisher('/scaled_point_cloud', PointCloud2, queue_size=10)
    rospy.spin()
