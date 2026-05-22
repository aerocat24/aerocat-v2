#!/usr/bin/env python3

import webbrowser
import rospy

def open_html():
    html_file_path = '/home/marcus/catkin_ws/src/tcc_marcus_albano/tello_pkg/scripts/rosbridge.html'
    webbrowser.open(html_file_path)

if __name__ == '__main__':
    rospy.init_node('open_html_node', anonymous=True)
    open_html()