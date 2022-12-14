使用说明
========
工程支持1台万集16线激光雷达数据显示
1. 配置相应的配置文件（16线：/wanji-master/wanji_pointcloud/launch/16e_points.launch），主要变量说明如下：
	device_ip：限定接收激光器ip（默认为空，不限定接收的激光器ip。设定后，将仅接收指定ip的数据）
	port：设置本机接收数据端口（多台8线激光器发送数据的目的端口都需要被设定为该端口号）
	rpm：电机转速（16线需按设定转速修改）
	data_length：数据长度（16线：1001）
2. 启动launch文件：
	16线：
		实时数据：roslaunch wanji_pointcloud 16e_points.launch
		回放pcap文件数据：roslaunch wanji_pointcloud 16e_points.launch pcap:=<path_to_pcapFile>
3. rosbag保存和回放：
	保存：rosbag record + 相应点云话题
	回放：rosbag play + .bag文件路径
