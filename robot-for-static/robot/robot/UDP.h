//#pragma once
//#include <iostream>
//#include <winsock2.h>
//#include "vision_detection.pb.h"
//#include "zss_cmd.pb.h"
//#include <string.h>
//#include <fstream>
//#include "zss_debug.pb.h"
//
//using namespace std;
//#pragma comment(lib, "WS2_32.lib")
//
//
//extern const int width = 4500;
//extern const int length = 6000;
//extern const int number = 8;
//
//class map_create
//{
//private:
//	int cost;
//	double width;
//	double length;
//public:
//	double resolution;
//	map_create()
//	{
//		cost = 0;
//		width = 4500;
//		length = 6000;
//		resolution = 0.1;
//	}
//	void set_cost();
//	void set_width(double w)
//	{
//		width = w;
//	}
//	void set_length(double l)
//	{
//		length = l;
//	}
//};
//
//void map_create::set_cost()
//{
//	SOCKET socket;
//	Vision_DetectionFrame map_;
//	//Recv(socket, map_);
//	//cout << floor(map_.robots_blue(3).x() * resolution) << endl;
//
//	//for (int i = 0; i < number; i++)
//	//{
//	//	int x = floor(map_.robots_blue(i).x() * resolution);
//	//	int y = floor(map_.robots_blue(i).y() * resolution);
//	//	cout << "(" << x << "," << y << ")" << endl;
//	//	for (int i = x - car_radius * resolution; i < x + car_radius * resolution; i++)
//	//	{
//	//		for (int j = y - car_radius * resolution; j < y + car_radius * resolution; j++)
//	//		{
//	//			map_cost[y][x] = 1;
//	//		}
//	//	}
//	//}
//	//for (int i = 0; i < map_.robots_yellow_size(); i++)
//	//{
//	//	int x = floor(map_.robots_yellow(i).x() * resolution);
//	//	int y = floor(map_.robots_yellow(i).y() * resolution);
//	//	for (int i = x - car_radius * resolution; i < x + car_radius * resolution; i++)
//	//	{
//	//		for (int j = y - car_radius * resolution; j < y + car_radius * resolution; j++)
//	//		{
//	//			map_cost[y][x] = 1;
//	//		}
//	//	}
//	//}
//	//int x = floor(map_.balls().x() * resolution);
//	//int y = floor(map_.balls().y() * resolution);
//	//map_cost[y][x] = 1;
//}
//
//
//class UDP
//{
//public:
//	//接收
//	void Recv(Vision_DetectionFrame rcr); //只接收一次数据
//	void Recv_map(Vision_DetectionFrame rcr); //读静态地图
//	//发送
//	void Send_motion(Robots_Command rcs, Robot_Command* rc, float id, float x, float y, float r, bool kick, float power, float spin);
//	//显示
//	void Send_debug(SOCKET s, Debug_Line* line, Point* start, Point* end, int x1, int y1, int x2, int y2);
//};
//
//void UDP::Recv(Vision_DetectionFrame rcr)
//{
//	WSADATA data;
//	WORD w = MAKEWORD(2, 2);
//	::WSAStartup(w, &data);
//	SOCKET s;
//	s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//	sockaddr_in addr;
//	char buff[100000] = {};
//	addr.sin_family = AF_INET;
//	addr.sin_port = htons(23333);
//	addr.sin_addr.s_addr = inet_addr("127.0.0.1");
//	::bind(s, (sockaddr*)&addr, sizeof(addr));
//	SOCKADDR_IN addr_Clt;
//	//cout << "UDP Server is Working!\n";
//	
//		int rec = recv(s, buff, 10000, 0);
//		cout << rec << endl;
//		//cout << inet_ntoa(addr.sin_addr) << " have linked!\n";
//		rcr.ParseFromArray(buff, sizeof(buff));
//		//cout << rcr.robots_yellow(1).x() << endl;
//	
//	::closesocket(s);
//	::WSACleanup();
//	system("pause");
//}
//
//void UDP::Recv_map(Vision_DetectionFrame rcr)
//{
//	map_create map;
//	WSADATA data; // 定义结构体变量
//	WORD w = MAKEWORD(2, 2); // 定义套接字版本
//	::WSAStartup(w, &data); // 初始化套接字库
//	SOCKET s;
//	s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  // 定义套接字类型
//	sockaddr_in addr; // 创建套接字地址类型
//	char buff[100000] = {};  // 接受数据缓冲区域
//	addr.sin_family = AF_INET;
//	addr.sin_port = htons(23333); // 设置ip地址变量端口
//	addr.sin_addr.s_addr = inet_addr("127.0.0.1");  //htonl(INADDR_ANY);// 设置ip地址
//	::bind(s, (sockaddr*)&addr, sizeof(addr)); // 绑定对应的地址跟端口号
//	SOCKADDR_IN addr_Clt;
//	//cout << "UDP Server is Working!\n";
//
//	int rec = recv(s, buff, 10000, 0);
//	cout << rec << endl;
//	//cout << inet_ntoa(addr.sin_addr) << " have linked!\n";
//	rcr.ParseFromArray(buff, sizeof(buff));
//	for (int i = 0; i < number; i++)
//	{
//		int x = floor(rcr.robots_blue(i).x() * map.resolution);
//		int y = floor(rcr.robots_blue(i).y() * map.resolution);
//		for (int i = int(x - car_radius * map.resolution-expand); i < int(x + car_radius * map.resolution+expand); i++)
//		{
//			for (int j = int(y - car_radius * map.resolution-expand); j < int(y + car_radius * map.resolution+expand); j++)
//			{
//				map_cost[j + 225][i + 300] = 1;
//			}
//		}
//	}
//	for (int i = 0; i < number; i++)
//	{
//		int x = floor(rcr.robots_yellow(i).x() * map.resolution);
//		int y = floor(rcr.robots_yellow(i).y() * map.resolution);
//		for (int i = int(x - car_radius * map.resolution-expand); i < int(x + car_radius * map.resolution+expand); i++)
//		{
//			for (int j = int(y - car_radius * map.resolution-expand); j < int(y + car_radius * map.resolution+expand); j++)
//			{
//				map_cost[j + 225][i + 300] = 1;
//			}
//		}
//	}
//	int x_ = floor(rcr.balls().x() * map.resolution);
//	int y_ = floor(rcr.balls().y() * map.resolution);
//	for (int i = int(x_ - ball_radius * map.resolution - expand); i < int(x_ + ball_radius * map.resolution + expand); i++)
//	{
//		for (int j = int(y_ - ball_radius * map.resolution - expand); j < int(y_ + ball_radius * map.resolution + expand); j++)
//		{
//			map_cost[j + 225][i + 300] = 1;
//		}
//	}
//
//}
//
//void UDP::Send_motion(Robots_Command rcs, Robot_Command* rc, float id, float x, float y, float r, bool kick, float power, float spin)
//{
//	rc = rcs.add_command();
//	rc->set_robot_id(id);
//	rc->set_velocity_x(x);
//	rc->set_velocity_y(y);
//	rc->set_velocity_r(r);
//	rc->set_kick(kick);
//	rc->set_power(power);
//	rc->set_dribbler_spin(spin);
//
//	int size = rcs.command_size();
//	char buffer[1024];
//	int buffer_size = 1024;
//	rcs.SerializeToArray(buffer, buffer_size);
//
//	WSADATA wsaData;
//	sockaddr_in RecvAddr;
//	int Port = 50001;
//
//	char SendBuf[1024 * 10];
//	int BufLen = 1024 * 10;
//
//	WSAStartup(MAKEWORD(2, 2), &wsaData);
//
//	SOCKET SendSocket;
//	SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//
//	RecvAddr.sin_family = AF_INET;
//	RecvAddr.sin_port = htons(Port);
//	RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
//
//	printf("Sending a datagram to the receiver...\n");
//
//	while (1)
//	{
//		cout << sendto(SendSocket, buffer, BufLen, 0, (SOCKADDR*)& RecvAddr, sizeof(RecvAddr));
//		//printf("finished sending,close socket.\n");
//		Sleep(100);
//	}
//	/*	rcs.clear_command();
//		rc = rcs.add_command();
//		rc->set_robot_id(0);
//		rc->set_velocity_r(0);
//		rc->set_velocity_x(0);
//		rc->set_velocity_y(0);
//		rc->set_kick(false);
//		rc->set_power(100);
//		rc->set_dribbler_spin(1);
//		rcs.SerializeToArray(buffer, buffer_size);
//*/
//	closesocket(SendSocket);
//	printf("Exting.\n");
//	WSACleanup();
//}
//
//
//void UDP::Send_debug(SOCKET s, Debug_Line* line, Point* start, Point* end, int x1, int y1, int x2, int y2)
//{
//
//	Debug_Msgs lines_set;
//	Debug_Msg* line_set;
//
//	line_set = lines_set.add_msgs();
//	line_set->set_type(Debug_Msg_Debug_Type_LINE);
//	line_set->set_color(Debug_Msg_Color_YELLOW);
//	line->set_allocated_start(start);
//	line->set_allocated_end(end);
//	line_set->set_allocated_line(line);
//
//	start->set_x(x1);
//	start->set_y(y1);
//	end->set_x(x2);
//	end->set_y(y2);
//	line->set_forward(true);
//	line->set_back(false);
//
//	char buffer[1024];
//	int buffer_size = 1024;
//	lines_set.SerializeToArray(buffer, buffer_size);
//
//	WSADATA wsaData;//初始化
//	sockaddr_in RecvAddr;//服务器地址
//	int Port = 20001;//服务器监听地址
//
//	char SendBuf[1024 * 10];//发送数据的缓冲区
//	int BufLen = 1024 * 10;//缓冲区大小
//
//	//初始化Socket
//	WSAStartup(MAKEWORD(2, 2), &wsaData);
//
//	//创建Socket对象
//	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//	//设置服务器地址
//	RecvAddr.sin_family = AF_INET;
//	RecvAddr.sin_port = htons(Port);
//	RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
//	//向服务器发送数据报
//	//printf("Sending a datagram to the receiver...\n");
//
//	cout << sendto(s, buffer, BufLen, 0, (SOCKADDR*)& RecvAddr, sizeof(RecvAddr)) << endl;
//	//发送完成，关闭Socket
//	//printf("finished sending,close socket.\n");
//	Sleep(100);
//}