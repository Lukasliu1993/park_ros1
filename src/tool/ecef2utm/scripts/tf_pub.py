import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
from pyproj import Proj, transform,Transformer
import tf
from geometry_msgs.msg import PoseStamped


def doMsg2(msg): 
    global pose_
    pose_x = pose_.pose.pose.position.x 
    pose_y = pose_.pose.pose.position.y 
    pose_z = pose_.pose.pose.position.z
    ori_x = pose_.pose.pose.orientation.x 
    ori_y = pose_.pose.pose.orientation.y
    ori_z = pose_.pose.pose.orientation.z
    ori_w = pose_.pose.pose.orientation.w

    # print(yaw)
    br = tf.TransformBroadcaster()

    #T1: map->odom; T2: odom->base_link, T3: map->base_link
    #T2*T1=T3 -> T1 = T2^-1 * T3

    #T2: odom->base_link
    T2_trans = (pose_x, pose_y, pose_z)
    T2_rota = (ori_x, ori_y, ori_z, ori_w)
    br.sendTransform(T2_trans,T2_rota,rospy.Time.now(),"base_link","odom")
    # (T2_trans,T2_rota) = listener.lookupTransform("odom", "base_link", rospy.Time(0))
    # print(T2_trans)
    # print(T2_rota)
    T2 = np.dot(tf.transformations.translation_matrix(T2_trans),
               tf.transformations.quaternion_matrix(T2_rota))
    T2_inverse = tf.transformations.inverse_matrix(T2)
    #T3: map->base_link
    T3_trans = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    T3_rota = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    T3 = np.dot(tf.transformations.translation_matrix(T3_trans),
               tf.transformations.quaternion_matrix(T3_rota))
    #T1: map->odom
    T1 = np.dot(T3, T2_inverse)
    # print(T1)
    T1_trans = tf.transformations.translation_from_matrix(T1)
    T1_rota = tf.transformations.quaternion_from_matrix(T1)
    br.sendTransform(T1_trans,T1_rota,rospy.Time.now(),"odom","map")
    # br.sendTransform(T3_trans,T3_rota,rospy.Time.now(),"base_link","map")

def doMsg(msg): 
    global pose_
    pose_ = msg

if __name__ == "__main__":
    global pose_
    pose_ = Odometry()
    rospy.init_node("tf_pub")
    rospy.loginfo("tf_pub!!!!")
    sub = rospy.Subscriber("/LIO_odom",Odometry,doMsg,queue_size=10)
    sub = rospy.Subscriber("/Fusion_pose",PoseStamped,doMsg2,queue_size=10)
    
    rospy.spin()