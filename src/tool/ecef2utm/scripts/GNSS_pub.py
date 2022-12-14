import numpy as np
import matplotlib.pyplot as plt
from roslib import message
from geometry_msgs.msg import PointStamped
import csv
import rospy
import socket
import time
import re
import math
import threading as th
import numpy as np



def get_pose(inputdata):
    byte_len = 2
    code = "b5620107"
    code_len = len(code)
    index = [substr.start() for substr in re.finditer(code, inputdata)]
    # print(index)
    for i in range(len(index)):
        length_1 = int(inputdata[index[i] + code_len:index[i] + code_len + 2], 16)
        length_2 = int(inputdata[index[i] + code_len + byte_len:index[i] + code_len + byte_len + 2], 16)
        length = 256 * length_2 + length_1
        # print('the length of the UBX-NAV-PVT:', length)
        start_idx = index[i] + 6 * byte_len
        #lontitude: 24 byte offset; 4 byte size
        lon_byte_offset = 24
        lon_idx = start_idx + lon_byte_offset * byte_len
        lon_1 = int(inputdata[lon_idx:lon_idx + 2], 16)
        lon_2 = int(inputdata[lon_idx + 1*byte_len:lon_idx + 1*byte_len +2], 16)
        lon_3 = int(inputdata[lon_idx + 2*byte_len:lon_idx + 2*byte_len +2], 16)
        lon_4 = int(inputdata[lon_idx + 3*byte_len:lon_idx + 3*byte_len +2], 16)
        lon = 16777216 * lon_4 + 65536 * lon_3 + 256 * lon_2 + lon_1
        lon = lon * 1e-7
        # print('lon byte is:', inputdata[lon_idx:lon_idx + 12])
        #latitude: 28 byte offset; 4 byte size
        lat_byte_offset = 28
        lat_idx = start_idx + lat_byte_offset * byte_len
        lat_1 = int(inputdata[lat_idx:lat_idx + 2], 16)
        lat_2 = int(inputdata[lat_idx + 1*byte_len:lat_idx + 1*byte_len +2], 16)
        lat_3 = int(inputdata[lat_idx + 2*byte_len:lat_idx + 2*byte_len +2], 16)
        lat_4 = int(inputdata[lat_idx + 3*byte_len:lat_idx + 3*byte_len +2], 16)
        lat = 16777216 * lat_4 + 65536 * lat_3 + 256 * lat_2 + lat_1
        lat = lat * 1e-7
        # print('the longtitude is:', lon)
        # print('the latitude is:', lat)
        lon_list.append(lon)
        lat_list.append(lat)
        #fixtype: 20 byte offset; 1 byte size
        fixtype_byte_offset = 20
        fixtype_idx = start_idx + fixtype_byte_offset * byte_len
        fixtype = int(inputdata[fixtype_idx:fixtype_idx + 2], 16)
        fix_type.append(fixtype)
        #fixstatus: 21 byte offset; 1 byte size
        fixstatus_byte_offset = 21
        fixstatus_idx = start_idx + fixstatus_byte_offset * byte_len
        fixstatus = int(inputdata[fixstatus_idx:fixstatus_idx + 2], 16)
        fix_status.append(fixstatus)

def get_time(inputdata):
    byte_len = 2
    code = "b5620107"
    code_len = len(code)
    index = [substr.start() for substr in re.finditer(code, inputdata)]
    for i in range(len(index)):
        length_1 = int(inputdata[index[i] + code_len:index[i] + code_len + 2], 16)
        length_2 = int(inputdata[index[i] + code_len + byte_len:index[i] + code_len + byte_len + 2], 16)
        length = 256 * length_2 + length_1
        # print('the length of the UBX-NAV-PVT:', length)
        start_idx = index[i] + 6 * byte_len
        #year: 4 byte offset; 2 byte size
        year_byte_offset = 4
        year_idx = start_idx + year_byte_offset * byte_len
        year_1 = int(inputdata[year_idx:year_idx + 2], 16)
        year_2 = int(inputdata[year_idx + 1*byte_len:year_idx + 1*byte_len +2], 16)
        year = 256 * year_2 + year_1
        # print('the year is:', year)
        #month: 6 byte offset; 1 byte size
        month_byte_offset = 6
        month_idx = start_idx + month_byte_offset * byte_len
        month = int(inputdata[month_idx:month_idx + 2], 16)
        #day: 7 byte offset; 1 byte size
        day_byte_offset = 7
        day_idx = start_idx + day_byte_offset * byte_len
        day = int(inputdata[day_idx:day_idx + 2], 16)
        #hour: 8 byte offset; 1 byte size
        hour_byte_offset = 8
        hour_idx = start_idx + hour_byte_offset * byte_len
        hour = int(inputdata[hour_idx:hour_idx + 2], 16)
        #minute: 9 byte offset; 1 byte size
        minute_byte_offset = 9
        minute_idx = start_idx + minute_byte_offset * byte_len
        minute = int(inputdata[minute_idx:minute_idx + 2], 16)
        #sec: 10 byte offset; 1 byte size
        sec_byte_offset = 10
        sec_idx = start_idx + sec_byte_offset * byte_len
        sec = int(inputdata[sec_idx:sec_idx + 2], 16)
        
        t = (year, month, day, hour + 8, minute, sec, 1, 48, 0)
        t = time.mktime(t)
        # print(time.strftime("%Y 年 %m 月 %d 日 %H:%M:%S", time.gmtime(t)))

        #nanosec: 12 byte offset; 4 byte size
        taccsec_byte_offset = 12
        taccsec_idx = start_idx + taccsec_byte_offset * byte_len
        taccsec_1 = int(inputdata[taccsec_idx:taccsec_idx + 2], 16)
        taccsec_2 = int(inputdata[taccsec_idx + 1*byte_len:taccsec_idx + 1*byte_len +2], 16)
        taccsec_3 = int(inputdata[taccsec_idx + 2*byte_len:taccsec_idx + 2*byte_len +2], 16)
        taccsec_4 = int(inputdata[taccsec_idx + 3*byte_len:taccsec_idx + 3*byte_len +2], 16)
        taccsec = 16777216 * taccsec_4 + 65536 * taccsec_3 + 256 * taccsec_2 + taccsec_1
        #nanosec: 16 byte offset; 4 byte size
        nanosec_byte_offset = 16
        nanosec_idx = start_idx + nanosec_byte_offset * byte_len
        nanosec_1 = int(inputdata[nanosec_idx:nanosec_idx + 2], 16)
        nanosec_2 = int(inputdata[nanosec_idx + 1*byte_len:nanosec_idx + 1*byte_len +2], 16)
        nanosec_3 = int(inputdata[nanosec_idx + 2*byte_len:nanosec_idx + 2*byte_len +2], 16)
        nanosec_4 = int(inputdata[nanosec_idx + 3*byte_len:nanosec_idx + 3*byte_len +2], 16)
        nanosec = 16777216 * nanosec_4 + 65536 * nanosec_3 + 256 * nanosec_2 + nanosec_1
        # print('the tacc time is:', t + nanosec * 1e-9)
        time_list.append(t + nanosec * 1e-9)

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


def send_msg(self):
    global frist_tag
    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`")
    recvData = client.recv(MaxBytes)
    recvData_buffer.append(recvData)
    if not recvData:
        print('接收数recvData据为空，我要退出了')
        return
    localTime = time.asctime( time.localtime(time.time()))
    # print(localTime, ' 接收到数据字节数:',len(recvData))
    get_pose(recvData.hex())
    get_time(recvData.hex())
    # print(recvData.hex())
    # print('the points number is: ', len(lon_list))

    if len(lon_list) != len(time_list):
        print('Length Error')
        print('lon_list length is : ', lon_list)
        print('time_list length is : ', time_list)
    if len(lon_list) == 0:
        # print("Wait For Rtk Data.......")
        return
    else:

        ref_point_lat = 34.3328626
        ref_point_lon = 108.782433
        last_lat = lat_list.pop(0)
        last_lon = lon_list.pop(0)
        x, y = GPStoXY(last_lat, last_lon, ref_point_lat, ref_point_lon)
        t = time_list.pop(0)
        t = rospy.Time.from_sec(t)
        fstatus = fix_status.pop(0)
                        
        point = PointStamped()
        point.point.x = x
        point.point.y = y
        point.point.z = fstatus
        point.header.stamp = rospy.Time.now()
        GNSS_pub.publish(point)


keep_going = True
CONSTANTS_RADIUS_OF_EARTH = 6371000.     # meters (m)
lon_list = []
lat_list = []
x_list = []
y_list = []
fix_type = []
fix_status = []
time_list = []
recvData_buffer = []
frist_tag = True
frist_xy = []

MaxBytes = 1024*1024
host = '10.0.2.1'
port = 20010
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(30)
client.connect((host, port))


rospy.init_node('listener', anonymous=True)
GNSS_pub = rospy.Publisher("/GNSS", PointStamped, queue_size=10)
rospy.Timer(rospy.Duration(0.1), send_msg)
rospy.spin()
