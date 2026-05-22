#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Polygon, Circle
import numpy as np

# Variáveis globais
boat_position = {"lat": 0.0, "lon": 0.0}

# Destinos
destination_lat = [-21.532936096191406, -21.532894134521484, -21.53277587890625, -21.532808303833008]
destination_lng = [-42.63374328613281, -42.63368606567383, 42.633785247802734, -42.63384246826172]

# Dimensões e parâmetros
one_degree_lat = 111139.0  # 1 grau de latitude em metros
one_degree_lon = 111139.0 * np.cos(np.radians(destination_lat[0]))  # Corrige longitude pela latitude
radius_in_degrees = 2 / one_degree_lat  # Raio de 2 metros em graus
boat_width_deg = 1.0 / one_degree_lon
boat_casco_height_deg = 1.5 / one_degree_lat
boat_triangle_height_deg = 1.0 / one_degree_lat
boat_casco_width_deg = 0.3 / one_degree_lon

# Status dos círculos
circle_status = [
    {"circle": None, "lat": lat, "lon": lon, "hit": False, "removal_counter": 0} 
    for lat, lon in zip(destination_lat, destination_lng)
]

# Leituras do GPS
gps_readings_count = 0

# Callback para o GPS
def gps_callback(data):
    global boat_position, gps_readings_count
    if len(data.data) >= 2:
        boat_position["lat"] = data.data[0]
        boat_position["lon"] = data.data[1]
        gps_readings_count += 1

def plot_boat():
    # Inicializa ROS
    rospy.init_node("boat_gps_visualizer", anonymous=True)
    rospy.Subscriber("/gps", Float32MultiArray, gps_callback)

    # Configuração do gráfico
    fig, ax = plt.subplots()
    ax.set_title("Controle de Posição do Barco")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.grid()

    # Limites do gráfico
    lat_min, lat_max = min(destination_lat), max(destination_lat)
    lng_min, lng_max = min(destination_lng), max(destination_lng)
    margin = radius_in_degrees * 10
    ax.set_xlim(lng_min - margin, lng_max + margin)
    ax.set_ylim(lat_min - margin, lat_max + margin)

    # Adiciona círculos de destino
    for circle_info in circle_status:
        circle = Circle((circle_info["lon"], circle_info["lat"]), radius_in_degrees, edgecolor="red", facecolor="none", lw=2)
        circle_info["circle"] = circle
        ax.add_patch(circle)

    # Configura o barco
    left_casco = Rectangle(
        (
            boat_position["lon"] - boat_width_deg / 2 - boat_casco_width_deg / 2,
            boat_position["lat"] - boat_casco_height_deg / 2,
        ),
        boat_casco_width_deg,
        boat_casco_height_deg,
        edgecolor="gray",
        facecolor="gray",
        lw=2,
    )
    right_casco = Rectangle(
        (
            boat_position["lon"] + boat_width_deg / 2 - boat_casco_width_deg / 2,
            boat_position["lat"] - boat_casco_height_deg / 2,
        ),
        boat_casco_width_deg,
        boat_casco_height_deg,
        edgecolor="gray",
        facecolor="gray",
        lw=2,
    )
    body_triangle = Polygon(
        [
            (boat_position["lon"], boat_position["lat"] + boat_triangle_height_deg / 2), 
            (boat_position["lon"] - boat_width_deg / 2, boat_position["lat"] - boat_triangle_height_deg / 2),
            (boat_position["lon"] + boat_width_deg / 2, boat_position["lat"] - boat_triangle_height_deg / 2),
        ],
        closed=True,
        edgecolor="black",
        facecolor="red",
        lw=1,
    )

    ax.add_patch(left_casco)
    ax.add_patch(right_casco)
    ax.add_patch(body_triangle)

    def update(frame):
        global gps_readings_count
        # Atualiza o barco
        left_casco.set_xy(
            (
                boat_position["lon"] - boat_width_deg / 2 - boat_casco_width_deg / 2,
                boat_position["lat"] - boat_casco_height_deg / 2,
            )
        )
        right_casco.set_xy(
            (
                boat_position["lon"] + boat_width_deg / 2 - boat_casco_width_deg / 2,
                boat_position["lat"] - boat_casco_height_deg / 2,
            )
        )
        body_triangle.set_xy(
            [
                (boat_position["lon"], boat_position["lat"] + boat_triangle_height_deg / 2),
                (boat_position["lon"] - boat_width_deg / 2, boat_position["lat"] - boat_triangle_height_deg / 2),
                (boat_position["lon"] + boat_width_deg / 2, boat_position["lat"] - boat_triangle_height_deg / 2),
            ]
        )

        # Atualiza os círculos
        for circle_info in circle_status:
            circle = circle_info["circle"]

            # Verifica se o círculo já foi removido
            if circle is None:
                continue

            if circle_info["hit"]:
                circle_info["removal_counter"] += 1
                if circle_info["removal_counter"] >= 10:
                    # Remova o círculo apenas se ainda existir
                    if circle is not None:
                        circle.remove()
                    circle_info["circle"] = None
                continue

            # Verifica se o barco está no raio do círculo
            distance = np.sqrt((boat_position["lat"] - circle_info["lat"]) ** 2 + (boat_position["lon"] - circle_info["lon"]) ** 2)
            if distance <= radius_in_degrees:
                circle.set_edgecolor("green")
                circle_info["hit"] = True

        return left_casco, right_casco, body_triangle

    ani = FuncAnimation(fig, update, interval=500)
    plt.show()

if __name__ == "__main__":
    try:
        plot_boat()
    except rospy.ROSInterruptException:
        pass
