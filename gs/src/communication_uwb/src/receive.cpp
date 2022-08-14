//UWB communication receiver
//编译：catkin_make -DCATKIN_WHITELIST_PACKAGES="communication_uwb"
//运行：rosrun communication_uwb communication_uwb_receive "/dev/ttyUSB0"


//引用库函数=====================================================================================
#include <stdio.h>
#include <stdlib.h>
//ROS发送消息
#include <ros/ros.h>
#include <std_msgs/String.h>
//串口
#include <fcntl.h>   //文件控制定义
#include <errno.h>   //错误号定义
#include <termios.h> //终端控制定义

//编码格式说明：
//0:				起始字节0xAA
//1~2:			数据长度srcLen (srcLen>0), uint16_t
//3~srcLen+2:		srcLen个原始数据, uint8_t
//srcLen+3~srcLen+4:	校验和(1~n+2字节数据求和)
//
#define DECODE_BUF_LEN 2048
uint8_t decodeBuf[DECODE_BUF_LEN];//解码缓存
uint32_t decodeBufWrite = 0;//解码缓存写入位置
uint32_t decodeBufRead = 0;//解码缓存处理位置

/**
  * @brief 编码（增加起始位'\n'和校验SUM）
  * @param src 要编码的数据
  * @param srcLen 要编码的数据长度(1~1021)
  * @param dst 编码后数据的存储位置
  * @retval int32_t 编码后的数据长度
  * @note 最大载荷长度1019字节
  */
uint16_t XR_encode(uint8_t* src, uint16_t srcLen, uint8_t* dst)
{
	uint32_t i;
	uint16_t sum = 0;
	dst[0] = 0xAA;
	*((uint16_t*)(&(dst[1]))) = srcLen;
	sum += dst[1];
	sum += dst[2];
	for(i = 0; i < srcLen; i++ )
	{
		dst[i+3] = src[i];
		sum += src[i];
	}
	*((uint16_t*)(&(dst[srcLen+3]))) = sum;
	return srcLen+5;
}

/**
  * @brief 解码（增加起始位'\n'和校验SUM）
  * @param src 要解码的数据
  * @param srcLen 要解码的数据长度(0~1021)
  * @param dst 解码后数据的存储位置
  * @retval int32_t 解码后的数据长度(为0表示解码失败，可能数据不完全)
  * @note 最大载荷长度1021字节
  */
uint16_t XR_decode(uint8_t* src, uint16_t srcLen, uint8_t* dst)
{
	uint32_t i;
	uint32_t start;
	uint32_t start2;
	uint16_t payloadLen;//载荷长度
	uint16_t sum1 = 0;
	uint16_t sum2;
	for(i = 0; i < srcLen; i++)
	{
		decodeBuf[decodeBufWrite%DECODE_BUF_LEN] = src[i];
		decodeBufWrite++;
	}
	while(1)
	{
		for(; decodeBufRead != decodeBufWrite && decodeBuf[decodeBufRead%DECODE_BUF_LEN] != 0xAA; decodeBufRead++);//寻找解码起始字符
		start = decodeBufRead;
		if(decodeBufWrite - start > 5)//缓存内有数据，尝试解码
		{
			payloadLen = (decodeBuf[(start+2)%DECODE_BUF_LEN] << 8) | decodeBuf[(start+1)%DECODE_BUF_LEN];//载荷长度
			if(payloadLen > 0 && payloadLen < 1020)//数据长度正确，尝试解码
			{
				if(decodeBufWrite - start >= payloadLen + 5)//缓存内已包含足够数据
				{
					sum2 = (decodeBuf[(start+payloadLen+4)%DECODE_BUF_LEN] << 8) | decodeBuf[(start+payloadLen+3)%DECODE_BUF_LEN];//校验和
					sum1 = 0;
					for(start2 = start + 1; start2 != decodeBufWrite - 2; start2++) sum1 += decodeBuf[start2%DECODE_BUF_LEN];
					if(sum1 == sum2)//校验和正确
					{
						for(i = 0; i < payloadLen; i++) dst[i] = decodeBuf[(decodeBufRead+i+3)%DECODE_BUF_LEN];
						decodeBufRead += payloadLen + 5;
						return payloadLen;
					}
					else decodeBufRead++;
				}
				else return 0;//缓存数据不足
			}
			else decodeBufRead++;//数据长度错误，丢弃该数据包
		}
		else return 0;
	}
	return 0;
}

class SerialPort
{
public:
    int recv(char *recvBuf, int bufLen)
    {
        int readLen = read(this->fd, recvBuf, bufLen); //非阻塞读取bufLen字节的数据
        if (readLen == -1 && errno == EAGAIN)
            return 0; //未读到数据
        return readLen;
    }
    int send(char *sendBuf, int bufLen)
    {
        int writeLen = write(this->fd, sendBuf, bufLen); //非阻塞发送bufLen字节的数据
        if (writeLen == -1 && errno == EAGAIN)
            return 0; //未读到数据
        return writeLen;
    }
    int fd;
    int success = 0;
    SerialPort(char *serialFile)
    {
        fd = open(serialFile, O_RDWR | O_NONBLOCK | O_NOCTTY); //以读写、非阻塞方式打开串口
        if (fd == -1)                                          //不能打开串口
        {
            perror("error when open port");
            return;
        }
        //设置串口通信速率:57600
        struct termios option;
        tcgetattr(fd, &option);                                      //读取当前参数
        cfsetispeed(&option, B921600);                               //设置输入速率
        cfsetospeed(&option, B921600);                               //设置输出速率
        option.c_iflag &= ~(INLCR | ICRNL | IGNCR);                  //不进行字符映射：不改变原始数据
        option.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); //不进行字符映射：不改变原始数据
        option.c_oflag &= ~(ONLCR | OCRNL);                          //不进行字符映射：不改变原始数据
        cfmakeraw(&option);                                          //设置为原始模式：不使用行缓存，实时读取所有输入数据
        tcflush(this->fd, TCIOFLUSH);                                //清空缓存，丢弃未接收字符
        if ((tcsetattr(this->fd, TCSANOW, &option)) != 0)            //激活新配置。TCSANOW：不等缓冲区数据读完
        {
            perror("com set error");
            return;
        }
        this->success = 1;
    }
    ~SerialPort(void)
    {
        if (this->fd != -1)
            close(fd);
    }
};

int main(int argc, char** argv)
{
	srand(time(0));
	uint8_t recvBuf[2048];
	uint8_t decodeBuf[2048];
	int recvLen = 0;
	printf("communication_uwb_receive start\r\n");
	//初始化ROS信息发送
	ros::init(argc, argv, "communication_uwb_receive");
	ros::NodeHandle my_node;
	ros::Publisher uwbRawPub = my_node.advertise<std_msgs::String>("/UWB_recv",100);
	ros::Rate rate(10);
	SerialPort port(argv[1]);
	while(ros::ok())//获取图像、解码并写缓冲区，涉及到异步IO
	{
		rate.sleep();
		ros::spinOnce();
		//testCodec();
		if((recvLen = port.recv((char*)recvBuf, 1024)) > 0)
		{
			int decodeLen = XR_decode(recvBuf, recvLen, decodeBuf);
			if(decodeLen > 0)
			{
				std_msgs::String strSend;
				strSend.data.resize(decodeLen);
				for(int i = 0; i < decodeLen; i++) strSend.data[i] = decodeBuf[i];
				uwbRawPub.publish(strSend);
				for(int i = 0; i < decodeLen; i++) printf("%c", decodeBuf[i]);
				printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");
			}
			//for(int i = 0; i < recvLen; i++) printf("%c", recvBuf[i]);
		}
	}
}



