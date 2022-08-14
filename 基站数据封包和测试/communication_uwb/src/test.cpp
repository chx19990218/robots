//UWB通信测试程序
//编译：catkin_make -DCATKIN_WHITELIST_PACKAGES="communication_uwb"
//运行：rosrun communication_uwb communication_uwb_test


//引用库函数=====================================================================================
#include <stdio.h>
#include <stdlib.h>
//ROS消息
#include <ros/ros.h>
#include <std_msgs/String.h>

std_msgs::String strRecv;
ros::Time timeBegin;
uint16_t stationMsgCntLast = 0;//最新的基站消息ID
void station_msgCallback(const std_msgs::String &msg)
{
	int strSize = msg.data.size();
	if(strSize == 12){//数据长度正确，处理
		uint16_t stationMsgCnt = *((uint16_t*)(&(msg.data[0])));
		uint16_t cntDiff = (uint16_t)(stationMsgCntLast - stationMsgCnt);
		if(cntDiff > 1000){//最新数据，可以处理
			printf("%d ", stationMsgCnt);
			for(int i = 0; i < 5; i++)//5架飞机
			{
				printf("[%d]: ", i);
				for(int j = 0; j < 2; j++)//2个字节数据
					for(int k = 0; k < 8; k++)//8个标志位
						printf("%d ", (msg.data[i*2+2+j] >> k) & 0x01);
			}
			printf("\r\n");
			strRecv = msg;
			stationMsgCntLast = stationMsgCnt;
		}
	}
	else{
		printf("[warning] [communication_uwb_test] [%f]: data length incorrect! expected 17, recv = %d\r\n", (ros::Time::now() - timeBegin).toSec(), strSize);
	}
}

int main(int argc, char** argv)
{
	printf("communication_uwb_test start\r\n");
	//初始化ROS
	ros::init(argc, argv, "communication_uwb_test");
	ros::NodeHandle my_node;
	ros::Subscriber station_msgSub = my_node.subscribe("/station_msg", 100, station_msgCallback);
	timeBegin = ros::Time::now();
	ros::spin();
}



