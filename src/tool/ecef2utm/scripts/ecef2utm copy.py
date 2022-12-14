import rospy
from nav_msgs.msg import Odometry
import numpy as np
import math
from pyproj import Proj, transform,Transformer
import tf
from geometry_msgs.msg import PoseStamped
transformer = Transformer.from_crs({"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},"epsg:32648")
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>
def catdis(p1x, p1y, p2x, p2y):
    global sum_d
    d = math.sqrt((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y))
    sum_d = d + sum_d

def catdeita(p1x, p1y, p2x, p2y):
    return d = math.sqrt((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y))


def doMsg(msg): 
    global x0, y0, z0, point_vec
    x, y, z = transformer.transform(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, radians=False)
    if len(point_vec) == 0: 
        point_vec.append([x, y]) 
    # else:
    #     catdis(point_vec[0][0], point_vec[0][1], x, y)
        # if  sum_d < 0.2:
        #     br = tf.TransformBroadcaster()
        #     (a1, a2, a3, a4) = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #     br.sendTransform((x - x0, y - y0, 0),(a1,a2,a3,a4),rospy.Time.now(),"base_link","map")
        #     pose = PoseStamped()
        #     pose.header.stamp = rospy.Time.now()
        #     pose.header.frame_id = "map"
        #     pose.pose.position.x = x - x0
        #     pose.pose.position.y = y - y0
        #     pose.pose.position.z = 0
        #     pose.pose.orientation.x = a1
        #     pose.pose.orientation.y = a2
        #     pose.pose.orientation.y = a2                                                                                          
        #     pose.pose.orientation.z = a3
        #     pose.pose.orientation.w = a4
        #     pub.publish(pose)
        #     point_vec.append([x, y])
        # else:
        #     yaw = np.arctan2((y - point_vec[0][1]),(x - point_vec[0][0]))
        #     br = tf.TransformBroadcaster()
        #     (a1, a2, a3, a4) = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #     br.sendTransform((x - x0, y - y0, 0),(a1,a2,a3,a4),rospy.Time.now(),"base_link","map")
        #     pose = PoseStamped()
        #     pose.header.stamp = rospy.Time.now()
        #     pose.header.frame_id = "map"
        #     pose.pose.position.x = x - x0
        #     pose.pose.position.y = y - y0
        #     pose.pose.position.z = 0
        #     pose.pose.orientation.x = a1
        #     pose.pose.orientation.y = a2
        #     pose.pose.orientation.z = a3
        #     pose.pose.orientation.w = a4
        #     pub.publish(pose)
        #     sum_d = 0 
        #     point_vec = []
        #     point_vec.append([x, y])
    else:
        point_vec.append([x, y])
        while  catdeita(point_vec[0][0], point_vec[0][1], point_vec[-1][0], point_vec[-1][1]) > 0.2 and len(point_vec) > 2:
            point_vec.pop(0)
        yaw = np.arctan2((point_vec[-1][1] - point_vec[0][1]),(point_vec[-1][0] - point_vec[0][0]))
        br = tf.TransformBroadcaster()
        (a1, a2, a3, a4) = tf.transformations.quaternion_from_euler(0, 0, yaw)
        br.sendTransform((x - x0, y - y0, 0),(a1,a2,a3,a4),rospy.Time.now(),"base_link","map")
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x - x0
        pose.pose.position.y = y - y0
        pose.pose.position.z = 0
        pose.pose.orientation.x = a1
        pose.pose.orientation.y = a2
        pose.pose.orientation.z = a3
        pose.pose.orientation.w = a4
        pub.publish(pose)



if __name__ == "__main__":
    global x0, y0, z0, point_vec
    point_vec = []
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    x_init_ = rospy.get_param("x_init",-1697472.2369)
    y_init_ = rospy.get_param("y_init",4991876.5218)
    z_init_ = rospy.get_param("z_init",3577508.9128)
    x0, y0, z0 = transformer.transform(x_init_, y_init_, z_init_, radians=False)
    pub = rospy.Publisher("/tracked_pose",PoseStamped,queue_size=10)
    sub = rospy.Subscriber("/fixposition/odometry",Odometry,doMsg,queue_size=10)
    rospy.spin()