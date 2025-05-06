#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import *
import cv2
import numpy as np
from std_msgs.msg import String
import geometry_msgs.msg
import yaml
from nav_msgs.msg import OccupancyGrid

def m_to_pixel(m_x, m_y):
    global resolution, origin, width, height
    
    pixel_x = round(((-origin[0]) + m_x) / resolution)
    pixel_y = round(((-origin[1]) - m_y) / resolution)
    
    pixel_x1 = round(((m_x - origin[1])/(-origin[1] - origin[1]))*width) 
    #gives more accurate values for y pixel coordinate
    pixel_y1 = round(((m_y + origin[1])/(origin[1] + origin[1]))*height) 
    
    return pixel_x, pixel_y1
    

def callback(data):
    global x_pts, y_pts
    pixel_x, pixel_y = m_to_pixel(data.point.x, data.point.y)
    x_pts.append(pixel_x)
    y_pts.append(pixel_y)
    

def capture_points():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/clicked_point", geometry_msgs.msg.PointStamped, callback)
    input("Press enter to continue...")

def set_height_and_width(msg):
    global width, height
    width = msg.info.width
    height = msg.info.height
    
def server_init():
    global resolution, origin
    rospy.wait_for_service('saveptsandgenpath')
    try:
        rospy.Subscriber('/map', OccupancyGrid, set_height_and_width)
        segment_name = input("enter segment name : ")
        resolution = rospy.get_param('/mapresolution', 0.05)
        origin = rospy.get_param('/maporigin', [0, 0, 0])
         
        capture_points()
        
        srvproxy = rospy.ServiceProxy('saveptsandgenpath', saveptsandgenpath)
        print(x_pts)
        print(y_pts)
        resp1 = srvproxy(x_pts, y_pts, segment_name)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

width = 0
height = 0
resolution = None
origin = None
x_pts = []
y_pts = []

if __name__ == "__main__":
    print(server_init())
