import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
from pyproj import Proj, transform,Transformer
import tf
from geometry_msgs.msg import PoseStamped
transformer = Transformer.from_crs({"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},"epsg:32648")


def doMsg(msg): 
    global x0, y0, z0, point_vec, deita_, listener
    (trans,rot) = listener.lookupTransform("ECEF", "FP_POI", rospy.Time(0))

    (r, p, yaw) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
    (a1, a2, a3, a4) = tf.transformations.quaternion_from_euler(0, 0, yaw )
    br = tf.TransformBroadcaster()
    br.sendTransform((trans[0] - x0, trans[1] - y0, trans[2] - z0),(a1, a2, a3, a4) ,rospy.Time.now(),"base_link","map")
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = trans[0] - x0
    pose.pose.position.y = trans[1] - y0
    pose.pose.position.z = trans[2] - z0
    pose.pose.orientation.x = a1
    pose.pose.orientation.y = a2
    pose.pose.orientation.z = a3
    pose.pose.orientation.w = a4
    pub.publish(pose)



if __name__ == "__main__":
    global x0, y0, z0, point_vec, deita_, listener
    point_vec = []
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    x_init_ = rospy.get_param("x_init",-1697472.2364)
    y_init_ = rospy.get_param("y_init",4991876.5244)
    z_init_ = rospy.get_param("z_init",3577508.9132)
    deita_ = rospy.get_param("deita",161.32569487052913)
    listener = tf.TransformListener()
    x0, y0, z0 = -1697472.2474,4991876.9097,3577508.4238
    pub = rospy.Publisher("/tracked_pose",PoseStamped,queue_size=10)
    sub = rospy.Subscriber("/fixposition/odometry",Odometry,doMsg,queue_size=10)
    rospy.spin()