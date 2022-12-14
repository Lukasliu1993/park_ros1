import numpy as np
import matplotlib.pyplot as plt
from roslib import message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
import rospy
import socket
import time
import re
import math
import threading as th
import numpy as np




def GPStoXY(lat, lon, ref_lat, ref_lon):
        # input GPS and Reference GPS in degrees
        # output XY in meters (m) X:North Y:East
        CONSTANTS_RADIUS_OF_EARTH = 6371000.
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)
        return x, -y


def send_msg(msg):
    ref_point_lat = 34.3328626
    ref_point_lon = 108.782433
    last_lat = msg.pose.pose.position.x
    last_lon = msg.pose.pose.position.y
    x, y = GPStoXY(last_lat, last_lon, ref_point_lat, ref_point_lon)
                    
    pose = PoseStamped()
    pose.pose.position.x = x 
    pose.pose.position.y = y 
    pose.pose.position.z = msg.twist.twist.linear.x
    pose.pose.orientation.x = msg.twist.twist.linear.y
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "gps"
    GNSS_pub.publish(pose)

rospy.init_node('listener', anonymous=True)
GNSS_pub = rospy.Publisher("/GNSS", PoseStamped, queue_size=10)
sub = rospy.Subscriber("/gps",Odometry,send_msg,queue_size=10)
rospy.spin()
