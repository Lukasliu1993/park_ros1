import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
from pyproj import Proj, transform,Transformer
import tf
from geometry_msgs.msg import PoseStamped


def doMsg(msg): 
    track_pose_.append(msg)
    if len(track_pose_) > 10:
        track_pose_.pop(0)

def OnTimer2(self):
    if len(track_pose_) == 0:
        return
    odom_ = Odometry()
    odom_.header.stamp = rospy.Time.now()
    odom_.header.frame_id = "LIO_odom"
    odom_.child_frame_id = "base_link"
    odom_.pose.pose.position.x = track_pose_[-1].pose.position.x
    odom_.pose.pose.position.y = track_pose_[-1].pose.position.y
    odom_.pose.pose.position.z = track_pose_[-1].pose.position.z
    odom_.pose.pose.orientation.x = track_pose_[-1].pose.orientation.x
    odom_.pose.pose.orientation.y = track_pose_[-1].pose.orientation.y
    odom_.pose.pose.orientation.z = track_pose_[-1].pose.orientation.z
    odom_.pose.pose.orientation.w = track_pose_[-1].pose.orientation.w
    pub.publish(odom_)
    return

if __name__ == "__main__":
    track_pose_ = []
    rospy.init_node("tracked2odom")
    rospy.loginfo("tracked2odom!!!!")
    pub = rospy.Publisher("/LIO_odom",Odometry,queue_size=10)
    sub = rospy.Subscriber("/tracked_pose",PoseStamped, doMsg,queue_size=10)
    rospy.Timer(rospy.Duration(0.05), OnTimer2)
    rospy.spin()