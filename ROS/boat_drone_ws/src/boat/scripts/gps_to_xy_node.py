# ORIGIN_LAT = -21.53269386291504   # <-- coloque sua latitude de referência (graus)
# ORIGIN_LON = -42.633811950683594 

#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

class GPSxyConverter:

    def __init__(self):
        rospy.init_node("gps_xy_converter", anonymous=False)

        self.x_sp = 10.0
        self.y_sp = 10.0

        self.current_theta = None
        self.x_rot = None
        self.y_rot = None

        # -------------------------
        # ORIGEM FIXA DO SISTEMA
        # -------------------------
        self.lat0 = -21.532712936401367 # coloque a origem desejada
        self.lon0 = -42.63380813598633 # coloque a origem desejada

        # -------------------------
        # ÂNGULO DE ROTAÇÃO (graus)
        # -------------------------
        self.theta_deg = 136 #140
        self.theta = math.radians(self.theta_deg)

        # fator de conversão aproximado
        self.meters_per_degree = 111319.9

        # subscreve ao tópico do GPS recebido como Float32MultiArray
        rospy.Subscriber("/boat/gps", Float32MultiArray, self.callback)
        rospy.Subscriber("/boat/compass", Int32, self.compass_callback)

        rospy.loginfo("GPS XY converter iniciado2.")

    def compass_callback(self, msg):
        self.compass_data = msg.data #compensaçao
        self.offset = 150
        self.current_theta = -(self.compass_data + self.offset)

        if(self.current_theta > 180):
            self.current_theta = self.current_theta - 360
        elif(self.current_theta < -180):
            self.current_theta = self.current_theta + 360
        # rospy.loginfo(f"compass recebido: {self.current_theta}")


    def callback(self, msg):

        if len(msg.data) < 2:
            rospy.logwarn("Float32MultiArray deve conter [lat, lon].")
            return

        lat = msg.data[0]
        lon = msg.data[1]

        # -------------------------
        # CONVERTER LAT/LON PARA XY
        # -------------------------
        x = (lon - self.lon0) * self.meters_per_degree * math.cos(
                math.radians((self.lat0 + lat) / 2)
            )
        y = (lat - self.lat0) * self.meters_per_degree

        # -------------------------
        # ROTAÇÃO DO SISTEMA
        # -------------------------
        self.x_rot = x * math.cos(self.theta) - y * math.sin(self.theta)
        self.y_rot = x * math.sin(self.theta) + y * math.cos(self.theta)

        # -------------------------
        # PRINTAR RESULTADO
        # -------------------------
        # rospy.loginfo("Lat: %.6f  Lon: %.6f  -->  X: %.3f m,  Y: %.3f m",
        #                lat, lon, self.x_rot, self.y_rot)
        dx = self.x_sp - self.x_rot
        dy = self.y_sp - self.y_rot

        self.ang_sp_rad = math.atan2(dy, dx)
        self.ang_sp_deg = math.degrees(self.ang_sp_rad)
        self.dist_sp = math.sqrt(dx**2 + dy**2)

        # rospy.loginfo("Setpoint --> Dist: %.3f m,  Angle: %.2f deg", self.dist_sp, self.ang_sp_deg)

    def run(self):
     
        rate = rospy.Rate(1) #10Hz -> 0.1s 

        rospy.loginfo("Iniciando loop de monitoramento...")

        while not rospy.is_shutdown():

            if self.x_rot is not None and self.y_rot is not None and self.current_theta is not None:
                rospy.loginfo(f"atual: X: {self.x_rot:.2f}, Y: {self.y_rot:.2f}, ang: {self.current_theta:.2f}")
                rospy.loginfo(f"desejado: X_sp: {self.x_sp:.2f}, Y_sp: {self.y_sp:.2f}, ang: {self.ang_sp_deg:.2f}")
                self.delta_theta = self.ang_sp_deg - self.current_theta
                if self.delta_theta > 180:
                    self.delta_theta -= 360
                elif self.delta_theta < -180:
                    self.delta_theta += 360
                rospy.loginfo(f"erro: dist: {self.dist_sp:.2f}, delta_ang: {self.delta_theta:.2f}")
            rate.sleep()

if __name__ == '__main__':

    try:
        gpsxyconverter = GPSxyConverter()
        gpsxyconverter.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass

