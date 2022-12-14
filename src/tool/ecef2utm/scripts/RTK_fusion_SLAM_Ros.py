import numpy as np
import math
import pickle
import random
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
import tf

def RTK_fusion_SLAM():
    global R_x_last, R_y_last, R_x_new, R_y_new, R_status_new
    global L_x_last, L_y_last, L_theta_last, L_x_new, L_y_new, L_theta_new
    global F_x_last, F_y_last, F_theta_last, F_x_new, F_y_new, F_theta_new
    global R_time_last, R_time_new
    max_trans = 0.5 #每1m最大修正0.1m
    max_rota = 0.05 # 每1弧度最大修正0.01弧度
    G_max = 1.0
    G_min = 1.0
    dt = rospy.Time.to_sec(R_time_new - R_time_last)
    if dt > 1.0:
        dt = 1.0
    use_RTK = 1.0
    local_coef = 1.0
    num_int_list = 10 #积分队列维持的长度,使用偶数
    #开始进行融合
    use_RTK = 1.0
    if R_status_new != 131:
        bad_RTK_x.append(R_x_new)
        bad_RTK_y.append(R_y_new)
        use_RTK = 0.0
        Int_rota.clear()
        Fix_rota.clear()
    #获取Local的平动和转动变化量
    delta_x = L_x_new - L_x_last
    delta_y = L_y_new - L_y_last
    delta_r = np.sqrt(delta_x * delta_x + delta_y * delta_y)
    delta_theta = L_theta_new - L_theta_last
    if delta_theta > np.pi:
        delta_theta = delta_theta - 2 * np.pi
    if delta_theta < -1 * np.pi:
        delta_theta = 2 * np.pi + delta_theta
    if delta_r == 0:
        d_theta = 0
    else:
        d_theta = np.arccos(delta_x / delta_r)
    if delta_y < 0:
        d_theta = -1 * d_theta
    change_theta = L_theta_last - d_theta
    dx_L = delta_r * np.cos(F_theta_last - change_theta)
    dy_L = delta_r * np.sin(F_theta_last - change_theta)
    dtheta_R = delta_theta
    #建立与RTK的“引力” G
    try:
        (trans,rot) = tf_listener.lookupTransform("map", "GNSS1", rospy.Time(0))
        x_last = trans[0]
        y_last = trans[1]
    except:
            x_last = F_x_last
            y_last = F_y_last
            print("****************RTK_fusion ERROR: no TF .****************")
            # return
    # print('the base_link to map is:', trans)
    # print('the F_x_last is ', F_x_last, ', F_y_last is ', F_y_last)
    # G_x = R_x_last - F_x_last
    # G_y = R_y_last - F_y_last
    
    G_x = R_x_last - x_last
    G_y = R_y_last - y_last
    G = np.sqrt(G_x ** 2 + G_y ** 2)
    #构建车体朝向的单位向量
    yaw_x = np.cos(F_theta_last)
    yaw_y = np.sin(F_theta_last)
    #得到引力产生的平动(点乘)和转动（叉乘）
    G_trans = np.dot([G_x, G_y], [yaw_x, yaw_y])
    G_rota_ = np.cross([yaw_x, yaw_y, 0], [G_x, G_y, 0])[-1]
    #更新历史的rota队列
    if use_RTK > 0:
        Int_rota.append(G_rota_)
        if len(Int_rota) > num_int_list:
            Int_rota.pop(0)
    G_rota = np.sum(Int_rota) / num_int_list

    #——非线性引力——
    #车朝向方向修正
    if np.abs(G_trans) > G_max:
        G_trans_strike = max_trans
        if G_trans < 0.0 :
            G_trans_strike = -1 * G_trans_strike
    else:
        G_trans_strike = (G_trans / G_max) ** 2 * max_trans
        if G_trans < 0.0 :
            G_trans_strike = -1 * G_trans_strike
    #车垂直方向修正
    if np.abs(G_rota) > G_max:
        G_trans_dip = max_trans
        if G_rota < 0.0 :
            G_trans_dip = -1 * G_trans_dip
    else:
        G_trans_dip = (G_rota / G_max) ** 2 * max_trans
        if G_rota < 0.0 :
            G_trans_dip = -1 * G_trans_dip
    #trans修正对应的坐标变换
    dx_G = (G_trans_strike * np.cos(F_theta_last) - G_trans_dip * np.sin(F_theta_last)) \
            * dt * use_RTK
    dy_G = (G_trans_strike * np.sin(F_theta_last) + G_trans_dip * np.cos(F_theta_last)) \
            * dt * use_RTK
    #记录历史的trans_dip从而对rota的积分修正，以保证yaw的计算不会和dip的修正耦合
    Dip_his.append(G_trans_dip)
    if use_RTK > 0:
        Fix_rota.append(np.sum(Dip_his) + G_rota_)
        if len(Fix_rota) > num_int_list:
            Fix_rota.pop(0)
    #车yaw角修正
    G_rota_yaw = 0.0
    if len(Fix_rota) == num_int_list:
        half_num = np.int32(num_int_list / 2)
#             diff_rota = np.sum(Int_rota[half_num : num_int_list]) - \
#                         np.sum(Int_rota[0:half_num])
        diff_rota = np.sum(Fix_rota[half_num : num_int_list]) - \
                    np.sum(Fix_rota[0:half_num])
        if np.abs(diff_rota) > G_max:
            G_rota_yaw = max_rota
            if diff_rota < 0.0 :
                G_rota_yaw = -1 * G_rota_yaw
        else:
            G_rota_yaw = (diff_rota / G_max) ** 2 * max_rota
            if diff_rota < 0.0 :
                G_rota_yaw = -1 * G_rota_yaw
    dtheta_G = G_rota_yaw * dt * use_RTK
#         if G > 1:
#             print("i={}".format(i))
#             print("dtheta_G={}".format(dtheta_G))
#             print("dx_G,dy_G={},{}".format(dx_G,dy_G))
    #融合修正位姿
    F_x_new = F_x_last + dx_L * local_coef + dx_G
    F_y_new = F_y_last + dy_L * local_coef + dy_G
    F_theta_new = F_theta_last + dtheta_R * local_coef + dtheta_G
    fusion_yaw_list.append(F_theta_new)

    Fusion_pose = PoseStamped()
    Fusion_pose.header.frame_id = "map"
    Fusion_pose.header.stamp = R_time_new
    Fusion_pose.pose.position.x = F_x_new
    Fusion_pose.pose.position.y = F_y_new
    
    r = R.from_euler('zyx', [F_theta_new, 0, 0])
    quater = r.as_quat()
    Fusion_pose.pose.orientation.x = quater[0]
    Fusion_pose.pose.orientation.y = quater[1]
    Fusion_pose.pose.orientation.z = quater[2]
    Fusion_pose.pose.orientation.w = quater[3]
    return Fusion_pose

def update_onlyodom():
    ###全局变量：各个传入的数据
    global R_x_last, R_y_last, R_x_new, R_y_new, R_status_new
    global L_x_last, L_y_last, L_theta_last, L_x_new, L_y_new, L_theta_new
    global F_x_last, F_y_last, F_theta_last, F_x_new, F_y_new, F_theta_new
    global R_time_last, R_time_new
    local_coef = 1.0
    #载入odom数据
    new_odom_msg = odom_msg_list.pop(0)
    L_x_new = new_odom_msg.pose.pose.position.x
    L_y_new = new_odom_msg.pose.pose.position.y
    q = new_odom_msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    euler = r.as_euler('xyz', degrees=False)
    L_theta_new = euler[2]
    odom_yaw_list.append(L_theta_new)
    #获取Local的平动和转动变化量
    delta_x = L_x_new - L_x_last
    delta_y = L_y_new - L_y_last
    delta_r = np.sqrt(delta_x * delta_x + delta_y * delta_y)
    delta_theta = L_theta_new - L_theta_last
    if delta_theta > np.pi:
        delta_theta = delta_theta - 2 * np.pi
    if delta_theta < -1 * np.pi:
        delta_theta = 2 * np.pi + delta_theta
    if delta_r == 0:
        d_theta = 0
    else:
        d_theta = np.arccos(delta_x / delta_r)
    if delta_y < 0:
        d_theta = -1 * d_theta
    change_theta = L_theta_last - d_theta
    dx_L = delta_r * np.cos(F_theta_last - change_theta)
    dy_L = delta_r * np.sin(F_theta_last - change_theta)
    dtheta_R = delta_theta
    #计算fusion并发布
    F_x_new = F_x_last + dx_L * local_coef
    F_y_new = F_y_last + dy_L * local_coef
    F_theta_new = F_theta_last + dtheta_R * local_coef
    fusion_yaw_list.append(F_theta_new)
    Fusion_pose = PoseStamped()
    Fusion_pose.header.frame_id = "map"
    Fusion_pose.header.stamp = new_odom_msg.header.stamp
    Fusion_pose.pose.position.x = F_x_new
    Fusion_pose.pose.position.y = F_y_new
    
    r = R.from_euler('zyx', [F_theta_new, 0, 0])
    quater = r.as_quat()
    Fusion_pose.pose.orientation.x = quater[0]
    Fusion_pose.pose.orientation.y = quater[1]
    Fusion_pose.pose.orientation.z = quater[2]
    Fusion_pose.pose.orientation.w = quater[3]
    
    Fusion_pose_pub.publish(Fusion_pose)
    Fusion_path_list.poses.append(Fusion_pose)
    #更新fusion和odom数据
    L_x_last, L_y_last, L_theta_last = L_x_new, L_y_new, L_theta_new
    F_x_last, F_y_last, F_theta_last = F_x_new, F_y_new, F_theta_new

def data_prepare():
    global init_flag
    ###全局变量：各个传入的数据
    global R_x_last, R_y_last, R_x_new, R_y_new, R_status_new
    global L_x_last, L_y_last, L_theta_last, L_x_new, L_y_new, L_theta_new
    global F_x_last, F_y_last, F_theta_last, F_x_new, F_y_new, F_theta_new
    global R_time_last, R_time_new
    ##用于控制输出的全局变量
    global print_flag_odom, print_flag_GNSS
    if len(odom_msg_list) == 0:
        if print_flag_odom:
            print("Warning:no odom msg, waiting for odom....")
            print_flag_odom = False
        return False
    else:
        print_flag_odom = True
    if len(GNSS_msg_list) == 0:
        if init_flag:
            update_onlyodom()
        if print_flag_GNSS:
#             print("Warning:no GNSS msg, waiting for GNSS....")
            print_flag_GNSS = False
        return False
    else:
        print_flag_GNSS = True
    ##对接收的数据进行预处理，把msg的信息转成融合数据格式
    #先对时间戳进行匹配
    min_dt = rospy.Duration.from_sec(0.06)  #两个数据的时间戳的最短间隔
    new_GNSS_msg = GNSS_msg_list.pop(0)
    new_odom_msg = odom_msg_list.pop(0)
    while True:
        odom_time = new_odom_msg.header.stamp
        GNSS_time = new_GNSS_msg.header.stamp
        time_diff = GNSS_time - odom_time
#         print("the while len(odom_msg_list) = ", len(odom_msg_list))
#         print("min_dt=", min_dt, ",and time_diff=", time_diff)
        if time_diff < min_dt and time_diff > -1 * min_dt:
            R_time_new = GNSS_time
            break
        elif time_diff >= min_dt:
            if len(odom_msg_list) > 0:
                new_odom_msg = odom_msg_list.pop(0)
                continue
            else:
                print("Warning: odom msg delay")
                return False
        else:
            if len(GNSS_msg_list) > 0:
                new_GNSS_msg = GNSS_msg_list.pop(0)
                continue
            else:
#                 print("Warning: GNSS msg delay")
                return False
    R_x_new = new_GNSS_msg.point.x
    R_y_new = new_GNSS_msg.point.y
    R_status_new = new_GNSS_msg.point.z
    
    L_x_new = new_odom_msg.pose.pose.position.x
    L_y_new = new_odom_msg.pose.pose.position.y
    q = new_odom_msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    euler = r.as_euler('xyz', degrees=False)
    L_theta_new = euler[2]
    odom_yaw_list.append(L_theta_new)
    
    return True

def data_update():
    ###全局变量：各个传入的数据
    global R_x_last, R_y_last, R_x_new, R_y_new, R_status_new
    global L_x_last, L_y_last, L_theta_last, L_x_new, L_y_new, L_theta_new
    global F_x_last, F_y_last, F_theta_last, F_x_new, F_y_new, F_theta_new
    global R_time_last, R_time_new
    #更新各个历史数据xx_last
    R_x_last, R_y_last = R_x_new, R_y_new
    L_x_last, L_y_last, L_theta_last = L_x_new, L_y_new, L_theta_new
    F_x_last, F_y_last, F_theta_last = F_x_new, F_y_new, F_theta_new
    R_time_last = R_time_new
    
    return
    
def init():
    ###全局变量：各个传入的数据
    global R_x_last, R_y_last, R_x_new, R_y_new, R_status_new
    global L_x_last, L_y_last, L_theta_last, L_x_new, L_y_new, L_theta_new
    global F_x_last, F_y_last, F_theta_last, F_x_new, F_y_new, F_theta_new
    global R_time_last, R_time_new
    
    #更新各个历史数据xx_last
    R_x_last, R_y_last = R_x_new, R_y_new
    L_x_last, L_y_last, L_theta_last = L_x_new, L_y_new, L_theta_new
    #初始化Fusion数据
    F_x_last, F_y_last, F_theta_last = L_x_new, L_y_new, L_theta_new
    R_time_last = R_time_new

    #
    Fusion_pose = PoseStamped()
    Fusion_pose.header.frame_id = "map"
    Fusion_pose.header.stamp = rospy.Time.now()
    Fusion_pose.pose.position.x = F_x_last
    Fusion_pose.pose.position.y = F_y_last
    
    r = R.from_euler('zyx', [F_theta_last, 0, 0])
    quater = r.as_quat()
    Fusion_pose.pose.orientation.x = quater[0]
    Fusion_pose.pose.orientation.y = quater[1]
    Fusion_pose.pose.orientation.z = quater[2]
    Fusion_pose.pose.orientation.w = quater[3]
    
    Fusion_pose_pub.publish(Fusion_pose)
    Fusion_path_list.poses.append(Fusion_pose)
    
    print("Initialize success")
    
    return

fusion_yaw_list = []
odom_yaw_list = []
odom_msg_list = []
GNSS_msg_list = []
bad_RTK_x = []
bad_RTK_y = []
RTK_history_x = []
RTK_history_y = []
bad_RTK_x = []
bad_RTK_y = []
Int_rota = []
Fix_rota = []
Dip_his = []
print_flag_odom = True
print_flag_GNSS = True
init_flag = False
Fusion_path_list = Path()
Odom_path_list = Path()
GNSS_path_list = Path()
Odom_pose_0_x = 0
Odom_pose_0_y = 0
Odom_pose_0_flag = False

def odom_msg(msg):
    global Odom_pose_0_x, Odom_pose_0_y, Odom_pose_0_flag
    buffer_size = 10
    odom_msg_list.append(msg)
    if len(odom_msg_list) > buffer_size:
        odom_msg_list.pop(0)
    #记录odom路径
    if Odom_pose_0_flag == False:
        Odom_pose_0_x = msg.pose.pose.position.x
        Odom_pose_0_y = msg.pose.pose.position.y
        Odom_pose_0_flag = True
    Odom_pose = PoseStamped()
    Odom_pose.pose.position.x = msg.pose.pose.position.x - Odom_pose_0_x
    Odom_pose.pose.position.y = msg.pose.pose.position.y - Odom_pose_0_y
    Odom_path_list.poses.append(Odom_pose)
    

def GNSS_msg(msg):
    buffer_size = 10
    GNSS_msg_list.append(msg)
    if len(GNSS_msg_list) > buffer_size:
        GNSS_msg_list.pop(0)
    #记录GNSS路径
    GNSS_pose = PoseStamped()
    GNSS_pose.pose.position.x = msg.point.x
    GNSS_pose.pose.position.y = msg.point.y
    GNSS_path_list.poses.append(GNSS_pose)
def OnTimer(self):
    global init_flag
    prepare_flag = data_prepare()
    
    if prepare_flag:
        if init_flag == False:
            init()
            init_flag = True
            return
        Fusion_pose = RTK_fusion_SLAM()
        Fusion_pose_pub.publish(Fusion_pose)
        Fusion_path_list.poses.append(Fusion_pose)
        data_update()
        
    #发布累计路径
    Fusion_path_list.header.frame_id = "map"
    Fusion_path_list.header.stamp = rospy.Time.now()
    Fusion_path_pub.publish(Fusion_path_list)
    
    Odom_path_list.header.frame_id = "map"
    Odom_path_list.header.stamp = rospy.Time.now()
    Odom_path_pub.publish(Odom_path_list)
#     print(Odom_path_list)
    
    GNSS_path_list.header.frame_id = "map"
    GNSS_path_list.header.stamp = rospy.Time.now()
    GNSS_path_pub.publish(GNSS_path_list)
    return

rospy.init_node('RTK_fusion',anonymous=True)
rospy.Subscriber("/LIO_odom", Odometry, odom_msg)
rospy.Subscriber("/GNSS", PointStamped, GNSS_msg)
# Fusion_pose_pub = rospy.Publisher("tracked_pose", PoseStamped, queue_size = 10)
Fusion_pose_pub = rospy.Publisher("Fusion_pose", PoseStamped, queue_size = 10)
Fusion_path_pub = rospy.Publisher("Fusion_path", Path, queue_size = 10)
Odom_path_pub = rospy.Publisher("Odom_path", Path, queue_size = 10)
GNSS_path_pub = rospy.Publisher("GNSS_path", Path, queue_size = 10)
tf_listener = tf.TransformListener()
rospy.Timer(rospy.Duration(0.1), OnTimer)
rospy.spin()