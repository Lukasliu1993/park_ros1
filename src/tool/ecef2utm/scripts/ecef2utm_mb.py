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
        (trans,rot) = listener.lookupTransform("ECEF", "FP_POI", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    x, y, z = transformer.transform(trans[0], trans[1], trans[2], radians=False)
    (r, p, yaw) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
    # print(yaw)
    q1 = tf.transformations.quaternion_from_euler(0, 0, 1 * (yaw + deita_ * np.pi /180) )
    br = tf.TransformBroadcaster()

    br.sendTransform((x - x0, y - y0, 0),q1,rospy.Time.now(),"base_link","map")
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x - x0
    pose.pose.position.y = y - y0
    pose.pose.position.z = 0
    pose.pose.orientation.x = q1[0]
    pose.pose.orientation.y = q1[1]
    pose.pose.orientation.z = q1[2]
    pose.pose.orientation.w = q1[3]
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
    x0, y0, z0 = transformer.transform(x_init_, y_init_, z_init_, radians=False)
    print(y0)
    pub = rospy.Publisher("/tracked_pose",PoseStamped,queue_size=10)
    sub = rospy.Subscriber("/fixposition/odometry",Odometry,doMsg,queue_size=10)
    rospy.spin()