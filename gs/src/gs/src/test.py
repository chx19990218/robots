# -*- coding: utf-8 -*-
#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String

#飞机状态数组
droneStatus = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]],dtype='uint8')


def talker():
	pub = rospy.Publisher('/station_msg', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	msgCnt = 0
	station_str = "123456789012"
	data = np.array([0,0,0,0,0,0,0,0,0,0,0,0],dtype='uint8')
	print(data)
	while not rospy.is_shutdown():#循环发送飞机状态
		msgCnt = msgCnt + 1
		data[0] = msgCnt % 256
		data[1] = (msgCnt / 256) % 256
		droneStatus[np.random.randint(0,5)][np.random.randint(0,16)] = np.random.randint(0,2)#随机修改一位数据
		for droneId in range(0,5):#编码所有数据到数组
			data[2+droneId*2] = 0
			data[3+droneId*2] = 0
			for bit in range(0,8):
				data[2+droneId*2] = data[2+droneId*2] | droneStatus[droneId][bit] << bit
				data[3+droneId*2] = data[3+droneId*2] | droneStatus[droneId][bit+8] << bit
		station_str = data.tostring()#数组转字符串
		pub.publish(station_str)#ROS发送字符串数据
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass