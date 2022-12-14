import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
from pyproj import Proj, transform,Transformer
import tf
from geometry_msgs.msg import PoseStamped,TransformStamped
transformer = Transformer.from_crs({"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},"epsg:32648")


def doMsg(msg): 
    global x0, y0, z0, point_vec, deita_, listener
    try:
        (trans,rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    LIO_odom = Odometry()
    LIO_odom.header.stamp = rospy.Time.now()
    LIO_odom.header.frame_id = "map"
    LIO_odom.pose.pose.position.x = trans[0]
    LIO_odom.pose.pose.position.y = trans[1]
    LIO_odom.pose.pose.position.z = trans[2]
    LIO_odom.pose.pose.orientation.x = rot[0]
    LIO_odom.pose.pose.orientation.y = rot[1]
    LIO_odom.pose.pose.orientation.z = rot[2]
    LIO_odom.pose.pose.orientation.w = rot[3]
    pub.publish(LIO_odom)

if __name__ == "__main__":
    global x0, y0, z0, point_vec, deita_, listener
    point_vec = []
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    listener = tf.TransformListener()
    pub = rospy.Publisher("/LIO_odom",Odometry, queue_size=10)
    sub = rospy.Subscriber("/odom",Odometry,doMsg,queue_size=10)
    rospy.spin()