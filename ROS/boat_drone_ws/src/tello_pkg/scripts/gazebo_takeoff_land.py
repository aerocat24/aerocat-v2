#!/usr/bin/env python3
"""
Script auxilar para executar pouso e decolagem do Tello na simulação.
"""
import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

isFlying = False
takingOff = False 
landing = False


def callback_takeoff(msg, callback_args):
    publisher = callback_args
    global isFlying, takingOff, landing
    takingOff = True

    if isFlying == False and takingOff == True: 
        isFlying = True
        vel_msg = Twist()
        vel_msg.linear.x = float(0)
        vel_msg.linear.y = float(0)
        vel_msg.linear.z = float(0.5)
        vel_msg.angular.z = float(0.0)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        publisher.publish(vel_msg)

def callback_land(msg, callback_args):
    publisher = callback_args
    global isFlying, takingOff, landing
    landing = True
    if isFlying == True and landing == True: 
        vel_msg = Twist()
        vel_msg.linear.x = float(0)
        vel_msg.linear.y = float(0)
        vel_msg.linear.z = float(-0.5)
        vel_msg.angular.z = float(0.0)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        publisher.publish(vel_msg)


def callback_sonar(msg, callback_args):
    publisher = callback_args
    global isFlying, takingOff, landing
    sensor = msg.range
    
    if sensor > 1.5 and isFlying == True and takingOff == True: 
        takingOff = False
        vel_msg = Twist()
        vel_msg.linear.x = float(0)
        vel_msg.linear.y = float(0)
        vel_msg.linear.z = float(0.0)
        vel_msg.angular.z = float(0.0)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        publisher.publish(vel_msg)
        print("Decolagem completa")
    if sensor < 0.03 and isFlying == True and landing == True: 
        vel_msg = Twist()
        vel_msg.linear.x = float(0)
        vel_msg.linear.y = float(0)
        vel_msg.linear.z = float(0.0)
        vel_msg.angular.z = float(0.0)
        vel_msg.angular.x = float(0.0)
        vel_msg.angular.y = float(0.0)
        landing = False
        isFlying = False
        publisher.publish(vel_msg)
        print("Pouso realizado com sucesso")

def main():
    rospy.init_node('aux_takeoff_land')

    pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
    
    rospy.Subscriber('/tello/takeoff', Empty, callback_takeoff, callback_args=(pub_cmd_vel))
    rospy.Subscriber('/tello/land', Empty, callback_land, callback_args=(pub_cmd_vel))
    rospy.Subscriber('/sonar_height', Range, callback_sonar, callback_args=(pub_cmd_vel))
    rospy.spin()


if __name__ == '__main__':
    main()