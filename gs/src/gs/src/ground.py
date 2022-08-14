# -*- coding: utf-8 -*-
#!/usr/bin/env python

import matplotlib.pyplot as plt
import os

import numpy as np
import Tkinter as tk
import math


import rospy
from std_msgs.msg import String
import threading

# 17 Byte in total
#Byte 0,1 ----> message id
#Byte 2,3,4 ----> first uav
#Byte 5,6,7 ----> second uav
#Byte 8,9,10 ----> third uav
#Byte 11,12,13 ----> fourth uav
#Byte 14,15,16 ----> fifth uav

##define obtaincontrol = 0x80

##define takeoff = 0x40
##define land = 0x20
##define rtl = 0x10
##define reboot = 0x08


global droneStatus
global label_name_array
global message
global plot_index
plot_index = 0
droneStatus = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]],dtype='uint8')

label_array = np.array([[0,1,0,0,0],[0,0,0,0,0]],dtype='uint8');






def all_obtain_control_callback():
	droneStatus[0][5] = 1
	droneStatus[1][5] = 1
	droneStatus[2][5] = 1
	droneStatus[3][5] = 1
	droneStatus[4][5] = 1

	
def all_release_control_callback():
	for i in range(5):
		for j in range(12):
			droneStatus[i][j] = 0
def obtain_control0():
	droneStatus[0][5] = 1 - droneStatus[0][5]
def obtain_control1():
	droneStatus[1][5] = 1 - droneStatus[1][5]
def obtain_control2():
	droneStatus[2][5] = 1 - droneStatus[2][5]
def obtain_control3():
	droneStatus[3][5] = 1 - droneStatus[3][5]
def obtain_control4():
	droneStatus[4][5] = 1 - droneStatus[4][5]
	
	
	
	
	
	
def all_fly_callback():
	if droneStatus[0][5] == 1:
		droneStatus[0][6] = 1
	if droneStatus[1][5] == 1:
		droneStatus[1][6] = 1
	if droneStatus[2][5] == 1:
		droneStatus[2][6] = 1
	if droneStatus[3][5] == 1:
		droneStatus[3][6] = 1
	if droneStatus[4][5] == 1:
		droneStatus[4][6] = 1
def all_land_callback():
	if droneStatus[0][5] == 1:
		droneStatus[0][6] = 0
	if droneStatus[1][5] == 1:
		droneStatus[1][6] = 0
	if droneStatus[2][5] == 1:
		droneStatus[2][6] = 0
	if droneStatus[3][5] == 1:
		droneStatus[3][6] = 0
	if droneStatus[4][5] == 1:
		droneStatus[4][6] = 0
def fly0():
	if droneStatus[0][5] == 1:
		droneStatus[0][6] = 1 - droneStatus[0][6]
def fly1():
	if droneStatus[1][5] == 1:
		droneStatus[1][6] = 1 - droneStatus[1][6]
def fly2():
	if droneStatus[2][5] == 1:
		droneStatus[2][6] = 1 - droneStatus[2][6]
def fly3():
	if droneStatus[3][5] == 1:
		droneStatus[3][6] = 1 - droneStatus[3][6]
def fly4():
	if droneStatus[4][5] == 1:
		droneStatus[4][6] = 1 - droneStatus[4][6]
	
	
	
	
	
	
	
	
def all_rtl_callback():
	if droneStatus[0][5] == 1:
		droneStatus[0][7] = 1
	if droneStatus[1][5] == 1:
		droneStatus[1][7] = 1
	if droneStatus[2][5] == 1:
		droneStatus[2][7] = 1
	if droneStatus[3][5] == 1:
		droneStatus[3][7] = 1
	if droneStatus[4][5] == 1:
		droneStatus[4][7] = 1
def all_outrtl_callback():
	if droneStatus[0][5] == 1:
		droneStatus[0][7] = 0
	if droneStatus[1][5] == 1:
		droneStatus[1][7] = 0
	if droneStatus[2][5] == 1:
		droneStatus[2][7] = 0
	if droneStatus[3][5] == 1:
		droneStatus[3][7] = 0
	if droneStatus[4][5] == 1:
		droneStatus[4][7] = 0
def rtl0():
	if droneStatus[0][5] == 1:
		droneStatus[0][7] = 1 - droneStatus[0][7]
def rtl1():
	if droneStatus[1][5] == 1:
		droneStatus[1][7] = 1 - droneStatus[1][7]
def rtl2():
	if droneStatus[2][5] == 1:
		droneStatus[2][7] = 1 - droneStatus[2][7]
def rtl3():
	if droneStatus[3][5] == 1:
		droneStatus[3][7] = 1 - droneStatus[3][7]
def rtl4():
	if droneStatus[4][5] == 1:
		droneStatus[4][7] = 1 - droneStatus[4][7]
	
	
def px_callback():
	global plot_index
	message.set('***********************************   We  will  plot  t-x   ***********************************')
	plot_index = 0
def py_callback():
	global plot_index
	message.set('***********************************   We  will  plot  t-y   ***********************************')
	plot_index = 1
def pz_callback():
	global plot_index
	message.set('***********************************   We  will  plot  t-z   ***********************************')
	plot_index = 2
def flight_state_callback():
	global plot_index
	message.set('*****************************   We  will  plot  t-flight_state   *****************************')
	plot_index = 3
def x_y_callback():
	global plot_index
	message.set('***********************************   We  will  plot  trajectory   ***********************************')
	plot_index = 4
def plot_callback():
	foldername = textinput.get()
	logPath = os.path.abspath('.')
	folderPath = logPath + '/' +foldername

	
	log_t_path = folderPath + '/' + 'log_t.txt'
	log_p_x_path = folderPath + '/' + 'log_p_x.txt'
	log_p_y_path = folderPath + '/' + 'log_p_y.txt'
	log_p_z_path = folderPath + '/' + 'log_p_z.txt'
	log_flight_state_path = folderPath + '/' + 'log_flight_state.txt'
	
	log_t = open(log_t_path, "r")
	log_p_x = open(log_p_x_path, "r")
	log_p_y = open(log_p_y_path, "r")
	log_p_z = open(log_p_z_path, "r")
	log_flight_state = open(log_flight_state_path, "r")
	
	t=[]
	lines = log_t.readlines()
	for line in lines:
		t.append(float(line))

	
	if plot_index == 0:
		real_x = []
		des_x = []
		lines = log_p_x.readlines()
		for line in lines:
			index = line.find('/')
			real_x.append(float(line[0:index]))
			des_x.append(float(line[index+1:-1]))

		if len(t) == len(real_x) & len(t) == len(des_x):
			plt.figure()
			plt.plot(t, real_x, 'r--')
			plt.plot(t, des_x, 'b')
			plt.grid()
			plt.xlabel('t/s')
			plt.ylabel('x/m')
			plt.legend(["real_x", "des_x"])
			plt.show()
	if plot_index == 1:
		real_y = []
		des_y = []
		lines = log_p_y.readlines()
		for line in lines:
			index = line.find('/')
			real_y.append(float(line[0:index]))
			des_y.append(float(line[index+1:-1]))

		if len(t) == len(real_y) & len(t) == len(des_y):
			plt.figure()
			plt.plot(t, real_y, 'r--')
			plt.plot(t, des_y, 'b')
			plt.grid()
			plt.xlabel('t/s')
			plt.ylabel('y/m')
			plt.legend(["real_y", "des_y"])
			plt.show()
	if plot_index == 2:
		real_z = []
		des_z = []
		lines = log_p_z.readlines()
		for line in lines:
			index = line.find('/')
			real_z.append(float(line[0:index]))
			des_z.append(float(line[index+1:-1]))

		if len(t) == len(real_z) & len(t) == len(des_z):
			plt.figure()
			plt.plot(t, real_z, 'r--')
			plt.plot(t, des_z, 'b')
			plt.grid()
			plt.xlabel('t/s')
			plt.ylabel('z/m')
			plt.legend(["real_z", "des_z"])
			plt.show()
	if plot_index == 3:
		flight_state = []
		lines = log_flight_state.readlines()
		for line in lines:
			flight_state.append(float(line))
			if len(t)==len(flight_state) :
				plt.figure()
				plt.plot(t, flight_state, 'r--')
				plt.grid()
				plt.xlabel('t/s')
				plt.ylabel('flight_state')
				plt.show()
	if plot_index == 4:
		real_x = []
		real_y = []
		lines = log_p_x.readlines()
		for line in lines:
			index = line.find('/')
			real_x.append(float(line[0:index]))
		lines = log_p_y.readlines()
		for line in lines:
			index = line.find('/')
			real_y.append(float(line[0:index]))
		if len(real_x) == len(real_y):
			plt.figure()
			plt.plot(real_x, real_y, 'r--')
			plt.grid()
			plt.xlabel('x/m')
			plt.ylabel('y/m')
			plt.show()

	
	
	
def gui():
	global message
	global textinput
	frame = tk.Tk()
	frame.title('Ground Station')
	img = tk.PhotoImage(file='hit2.png')	
	
	
	w = img.width()
	h = img.height()
	frame.geometry('%dx%d+0+0' % (w,h))
	
	
	bigmenu = tk.Menu(frame, fg = 'white', bg = 'gray', font="Helvetica -23 bold")
	datamenu = tk.Menu(bigmenu, tearoff = 0)
	datamenu.add_command(label='x', command = px_callback)
	datamenu.add_command(label='y', command = py_callback)
	datamenu.add_command(label='z', command = pz_callback)
	datamenu.add_separator()
	datamenu.add_command(label='flight_state', command = flight_state_callback)
	datamenu.add_separator()
	datamenu.add_command(label='trajectory', command = x_y_callback)
	bigmenu.add_cascade(label = 'Log', menu = datamenu)
	frame.config(menu = bigmenu)
	
	
	
	
	#	Label
	background_label = tk.Label(frame, 
															image = img).place(x=0, y=0, relwidth=1, relheight=1)
	
	message = tk.StringVar()
	message.set('***************************   Welcome  to  HITCSC  Ground  Station   ***************************')
	
	
	top_label = tk.Label(frame, 
											 textvariable = message, 
											 width = 80, height = 2,
											 fg="red", bg = "DarkBlue",
											 font="Helvetica -23 bold").place(x=0,y=30)
											 
	bottom_label = tk.Label(frame, 
											 text="********* Honor  Industriousness  Transcension  Cooperation  Steadfastness  Champion *********", 
											 width = 80, height = 2,
											 fg="red", bg = "DarkBlue",
											 font="Helvetica -23 bold").place(x=0,y=610)
											 
#	top_label = tk.Label(frame, 
#											 text="Reboot : ", 
#											 width = 18, height = 1,
#											 fg="red", bg = "LemonChiffon",
#											 font="Helvetica -20 bold").place(x=30,y=285)
#											 
#											 
#	top_label = tk.Label(frame, 
#											 text="Obtain  control : ", 
#											 width = 18, height = 1,
#											 fg="red", bg = "LemonChiffon",
#											 font="Helvetica -20 bold").place(x=30,y=356)
#											 
#											 
#	top_label = tk.Label(frame, 
#											 text="Take  off : ", 
#											 width = 18, height = 1,
#											 fg="red", bg = "LemonChiffon",
#											 font="Helvetica -20 bold").place(x=30,y=424)
											 
	
	
	
	#	Obtain Control Button
	b = tk.Button(frame, 
						text="All  obtain  control",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_obtain_control_callback).place(x=30,y=150)

						
	tk.Button(frame, 
						text="All  release  control",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_release_control_callback).place(x=770,y=150)
						
	tk.Checkbutton(frame, 
						text="0",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = obtain_control0).place(x=380,y=150)
	
	tk.Checkbutton(frame, 
						text="1",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = obtain_control1).place(x=460,y=150)
						
	tk.Checkbutton(frame, 
						text="2",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = obtain_control2).place(x=540,y=150)
						
	tk.Checkbutton(frame, 
						text="3",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = obtain_control3).place(x=620,y=150)
						
	tk.Checkbutton(frame, 
						text="4",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = obtain_control4).place(x=700,y=150)
						
						
						
					






	#	Takeoff or Land Button
	tk.Button(frame, 
						text="All  fly",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_fly_callback).place(x=30,y=210)
						
	tk.Button(frame, 
						text="All  land",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_land_callback).place(x=770,y=210)
	tk.Checkbutton(frame, 
						text="0",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = fly0).place(x=380,y=210)
	
	tk.Checkbutton(frame, 
						text="1",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = fly1).place(x=460,y=210)
						
	tk.Checkbutton(frame, 
						text="2",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = fly2).place(x=540,y=210)
						
	tk.Checkbutton(frame, 
						text="3",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = fly3).place(x=620,y=210)
						
	tk.Checkbutton(frame, 
						text="4",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = fly4).place(x=700,y=210)
						
						
						
						
						
						
						
						
	#	Rtl Button
	tk.Button(frame, 
						text="All  rtl",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_rtl_callback).place(x=30,y=270)
						
	tk.Button(frame, 
						text="All  out  rtl",
						fg="green", bg="LemonChiffon", 
						font="Helvetica -20 bold",
						width=25,height=1, 
						command=all_outrtl_callback).place(x=770,y=270)
						
	tk.Checkbutton(frame, 
						text="0",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = rtl0).place(x=380,y=270)
	
	tk.Checkbutton(frame, 
						text="1",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = rtl1).place(x=460,y=270)
						
	tk.Checkbutton(frame, 
						text="2",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = rtl2).place(x=540,y=270)
						
	tk.Checkbutton(frame, 
						text="3",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = rtl3).place(x=620,y=270)
						
	tk.Checkbutton(frame, 
						text="4",
						fg="LemonChiffon", bg="blue", 
						font="Helvetica -20 bold",
						indicatoron = False,
						selectcolor = "red",
						width=3,height=1, 
						command = rtl4).place(x=700,y=270)
						

	state = ["*****","ERROR"]
	rtk_message = tk.StringVar()
	rtk_message.set("     UAV0   :     %s     |   UAV1   :     %s     |   UAV2   :     %s     |   UAV3   :     %s     |   UAV4   :     %s     " % (state[label_array[0][0]] , state[label_array[0][1]] , state[label_array[0][2]] , state[label_array[0][3]] , state[label_array[0][4]]))
	
	
	
	battery_message = tk.StringVar()
	battery_message.set("     UAV0   :     %s     |   UAV1   :     %s     |   UAV2   :     %s     |   UAV3   :     %s     |   UAV4   :     %s     " % (state[label_array[1][0]] , state[label_array[1][1]] , state[label_array[1][2]] , state[label_array[1][3]] , state[label_array[1][4]]))


	label = tk.Label(frame, 
									text="RTK state : ", 
									font="Helvetica -18 bold",
									fg="red", bg="LemonChiffon",
									width=12,height=1).place(x=40,y=350)
									
	label = tk.Label(frame, 
									text="Battery state : ", 
									font="Helvetica -18 bold",
									fg="red", bg="LemonChiffon",
									width=12,height=1).place(x=40,y=400)



	label = tk.Label(frame, 
									textvariable=rtk_message, 
									font="Helvetica -18 bold",
									fg="red", bg="LemonChiffon").place(x=200,y=350)
		
	label = tk.Label(frame, 
									textvariable=battery_message, 
									font="Helvetica -18 bold",
									fg="red", bg="LemonChiffon").place(x=200,y=400)
									
									
	textinput = tk.Entry(frame, width=25,show=None)
	textinput.place(x=750, y=550)
	
	tk.Button(frame, text = 'Plot', width = 7, height = 1, command = plot_callback).place(x=1000, y=550)


	frame.mainloop()
	

	
t1 = threading.Thread(target = gui)
t1.start()

pub = rospy.Publisher('/UWB_send', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz
msgCnt = 0
station_str = "123456789012"
data = np.array([0,0,0,0,0,0,0,0,0,0,0,0],dtype='uint8')

while not rospy.is_shutdown():#循环发送飞机状态如果daemon属性是False，线程不会随主线程的结束而结束，这时如果线程访问主线程的资源，就会出错
	msgCnt = msgCnt + 1
	data[0] = msgCnt % 256
	data[1] = (msgCnt / 256) % 256
	for droneId in range(0,5):#编码所有数据到数组
		data[2+droneId*2] = 0
		data[3+droneId*2] = 0
		for bit in range(0,8):
			data[2+droneId*2] = data[2+droneId*2] | droneStatus[droneId][bit] << bit
			data[3+droneId*2] = data[3+droneId*2] | droneStatus[droneId][bit+8] << bit
	station_str = data.tostring()#数组转字符串
	pub.publish(station_str)#ROS发送字符串数据
	rate.sleep()
