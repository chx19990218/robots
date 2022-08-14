基于UWB的机间通信系统 V0.1 测试版

系统采用广播、异步收发形式，每架飞机搭载两个UWB模块，一个用于发送(广播)，一个用于接收(监听)；
无回传校验及内部缓冲区，会丢包，丢包率随场景和负载变动，无干扰及遮挡时通信距离实测最远180米；
理论极限空中总带宽为6Mbps。为保证系统稳定性，推荐带宽占用低于100KBps，即最多十架飞机分别以20Hz频率广播500字节长度数据包；
理论延迟约2ms，实际延迟受限于操作系统，待测试；
注意！UWB模块分发送与接收两种，固件不同，不可混用

ROS包组成：
	接收节点：communication_uwb_receive
		命令行启动：rosrun communication_uwb communication_uwb_receive /dev/ttyUSB0
		需修改串口号，后续计划使用udev绑定串口号
		发布ROS消息："/UWB_recv"，同时将收到的数据打印于屏幕便于调试
	发送节点：communication_uwb_transmit
		命令行启动：rosrun communication_uwb communication_uwb_transmit /dev/ttyUSB1
		需修改串口号，后续计划使用udev绑定串口号
		接收ROS消息："/UWB_send"，同时将发送的数据打印于屏幕便于调试

测试流程：
	电脑A用于接收数据，电脑B用于发送数据
	1.启动系统：
		(电脑A)命令行运行：roscore
		(电脑A)命令行运行：rosrun communication_uwb communication_uwb_receive /dev/ttyUSB0
		(电脑B)命令行运行：roscore
		(电脑B)命令行运行：rosrun communication_uwb communication_uwb_transmit /dev/ttyUSB0
	2.监听消息：
		(电脑A)命令行运行：rostopic echo /UWB_recv
	3.发布消息：
		(电脑B)命令行运行：rostopic pub -r 10 /UWB_send std_msgs/String "data: 'asrafrgdr'"
	4.现象：
		(电脑A)监听ROS消息"/UWB_recv"时，出现10Hz发布的字符串'asrafrgdr'




邢锐
2021-07-03