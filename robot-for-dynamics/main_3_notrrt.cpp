#include "rrt.h"
#include <iostream>
#include "vision_detection.pb.h"
#include <string.h>
#include <fstream>
#include <vector>
#include "udp_commu_recv.h"
#include "udp_commu_send.h"
#include <math.h>
#include <ctime>

int EXPAND = 20;
int Max_vel = 400;
int expand;
using namespace std;
#pragma comment(lib, "WS2_32.lib")

//地图和速度控制参数
const double car_radius = 90;
const double ball_radius = 50;

const double resolution = 0.1;
int map_cost[450][600];
int map_blue_obstacles_x[8];
int map_blue_obstacles_y[8];
int map_yellow_obstacles_x[8];
int map_yellow_obstacles_y[8];
int test_car_number = 0;
int number_map_blue_obstacles = 0;
int number_map_yellow_obstacles = 0;

double offset = 7;
double lethal_distance = 40.0;


void setpath(Debug_Msgs lines_set, int* x_path, int* y_path, int number) {
	for (int i = 0; i < number; i++)
	{
		Debug_Msg* line_set = new Debug_Msg();
		line_set = lines_set.add_msgs();
		line_set->set_type(Debug_Msg_Debug_Type_LINE);
		line_set->set_color(Debug_Msg_Color_BLUE);
		Debug_Line* line = new Debug_Line();
		Point* start = new Point();
		Point* end = new Point();

		start->set_x(x_path[i] - 300);
		start->set_y(y_path[i] - 225);
		end->set_x(x_path[i + 1] - 300);
		end->set_y(y_path[i + 1] - 225);

		line->set_allocated_start(start);
		line->set_allocated_end(end);
		line->set_forward(true);
		line->set_back(true);
		line_set->set_allocated_line(line);
	}

	char* buffer = new char[10240];
	int buffer_size = 10240;
	lines_set.SerializeToArray(buffer, buffer_size);

	WSADATA wsaData;
	sockaddr_in RecvAddr;
	int Port = 20001;
	SOCKET Socket;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_port = htons(Port);
	RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	cout << sendto(Socket, buffer, buffer_size, 0, (SOCKADDR*)& RecvAddr, sizeof(RecvAddr)) << endl;
}

double Get_angle(int x1, int y1, int x2, int y2) {
	double angle = (y2 - y1) / (x2 - x1);
	return angle;
}

int Get_distance(int x1, int y1, int x2, int y2) {
	int distance = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	return distance;
}

double Get_dis_point_to_line(int cur_x, int cur_y, int end_x, int end_y, float alpha) {
	if (abs(alpha - 3.14159 / 2) < 0.1)
		return abs(cur_x - end_x);
	else {
		float k = tan(alpha);
		float b = end_y - k * end_x;
		float dis = abs(k*cur_x - cur_y + b) / sqrt(k*k + 1);
		return dis;
	}
}

//double* setv(int curx, int cury, int goalx, int goaly, double oritation)
//{
//	double *vel = new double[2];
//	int dx = goalx - curx;
//	int dy = goaly - cury;
//	double alpha = atan2(dy, dx);
//
//	double t_glb_vx;
//	double t_glb_vy;
//	double dis = sqrt((curx - goalx)*(curx - goalx) + (cury - goaly)*(cury - goaly));
//
//	if (dis <= 16)
//	{
//		t_glb_vx = 10 * (goalx - curx);
//		t_glb_vy = 10 * (goaly - cury);
//	}
//	else
//	{
//		t_glb_vx = speed * cos(alpha);
//		t_glb_vy = speed * sin(alpha);
//	}
//
//	double t_vx = t_glb_vy * sin(oritation) + t_glb_vx * cos(oritation);
//	double t_vy = t_glb_vy * cos(oritation) - t_glb_vx * sin(oritation);
//	vel[0] = t_vx;
//	vel[1] = t_vy;
//	return vel;
//}

double run_away_velocity(double cur_x, double cur_y, double nearest_point_x, double nearest_point_y) {
	double run_away_velocity;
	int repulsion = 20000;
	double danger_distance = 20;
	double distance = sqrt(pow(nearest_point_x - cur_x, 2) + pow(nearest_point_y - cur_y, 2)) - 15;
	run_away_velocity = repulsion / pow(distance, 3);
	return run_away_velocity;
}

double* setv(double cur_x, double cur_y, int start_x, int start_y, int goal_x, int goal_y, double oritation, int flag)
{
	double *vel = new double[2];
	float check = 200;
	int acc_down = 50;
	int speed = Max_vel;

	int dx = goal_x - cur_x;
	int dy = goal_y - cur_y;
	float temp_vx;
	float temp_vy;
	float dv_x, dv_y;

	float alpha = atan2(dy, dx);

	double temp_dis1 = Get_distance(cur_x, cur_y, start_x, start_y);
	double temp_dis = Get_distance(cur_x, cur_y, goal_x, goal_y);
	double distance = Get_distance(start_x, start_y, goal_x, goal_y);
	double min_dis = 100;
	if (flag == 1) {
		if (temp_dis > distance / 2) {
			//	speed = sqrt(800 * abs(distance - temp_dis)) + 50;//+50
			//	if (distance - temp_dis > 100)
			speed = 250;//350
		}
		else {
			speed = sqrt(60 * temp_dis);//+50
			if (temp_dis > 100)
				speed = 250;//250
		}
	}
	if (flag == 0) {
		if (temp_dis > distance / 2) {
			//speed = sqrt(800 * abs(distance - temp_dis)) + 20;//+20
			//if (distance - temp_dis > 100)
			speed = 250;//350
		}
		else {
			speed = sqrt(60 * temp_dis) + 50;//+50
			if (temp_dis > 100)
				speed = 250;//250
		}
	}
	if (flag == 2) {
		double dx_2 = cur_x - start_x;
		double dy_2 = cur_y - start_y;
		double temp_dis2 = Get_distance(cur_x, cur_y, start_x, start_y);
		double alpha_2 = atan2(dy_2, dx_2);

		speed = sqrt(60 * temp_dis);
		double speed_2 = sqrt(800 * abs(100 - temp_dis)) + 20;

		vel[0] = cos(alpha - oritation) * speed + cos(alpha_2 - oritation) * speed_2;
		vel[1] = sin(alpha - oritation) * speed + sin(alpha_2 - oritation) * speed_2;
	}
	//cout << "speeed  " << speed << endl;
	temp_vx = cos(alpha - oritation) * speed;
	temp_vy = sin(alpha - oritation) * speed;

	float cur_dis = Get_dis_point_to_line(cur_x, cur_y, goal_x, goal_y, alpha);
	double alpha_1 = atan2(cur_y - start_y, cur_x - start_x);
	int cof;
	if (alpha_1 > alpha) {
		cof = 1;
	}
	else if (alpha_1 < alpha) {
		cof = -1;
	}
	else {
		cof = 0;
	}
	dv_x = cof * check * cur_dis * sin(alpha - oritation);
	dv_y = -cof * check * cur_dis * cos(alpha - oritation);

	//vel[0] = dv_x + temp_vx;
	//vel[1] = dv_y + temp_vy;

	vel[0] = temp_vx;
	vel[1] = temp_vy;
	return vel;
}

//double* seta(int curx, int cury, int goalx, int goaly, double oritation)
//{
//	double accl[2];
//	const int acc = 10;
//	int dx = goalx - curx;
//	int dy = goaly - cury;
//	double alpha = atan2(dy, dx);
//
//	double t_glb_ax;
//	double t_glb_ay;
//	double dis = sqrt((curx - goalx)*(curx - goalx) + (cury - goaly)*(cury - goaly));
//
//	if (dis <= 16)
//	{
//		t_glb_ax = 10 * (goalx - curx);
//		t_glb_ay = 10 * (goaly - cury);
//	}
//	else
//	{
//		t_glb_ax = acc * cos(alpha);
//		t_glb_ay = acc * sin(alpha);
//	}
//
//	double t_ax = t_glb_ay * sin(oritation) + t_glb_ax * cos(oritation);
//	double t_ay = t_glb_ay * cos(oritation) - t_glb_ax * sin(oritation);
//	accl[0] = t_ax;
//	accl[1] = t_ay;
//	return accl;
//}

//void motion_control(udp_commu_send udp_send, float id, int curx, int cury, int startx, int starty, int goalx, int goaly, double oritation)
//{
//	int step = 5;
//	double distance = (curx - goalx)*(curx - goalx) + (cury - goaly)*(cury - goaly);
//	double path = (curx - startx)*(curx - startx) + (cury - starty)*(cury - starty);
//	double* vel = setv(curx, cury, goalx, goaly, oritation);
//	double* accl = seta(curx, cury, goalx, goaly, oritation);
//	while (distance >= 0, 2 * path)
//	{
//		for (int i = 0; i < step; i++)
//		{
//			udp_send.send(id, vel[0] + i * accl[0], vel[1] + i * accl[1], 0, false, 100, 1);
//		}
//	}
//	while (0.2 * path <= distance <= 0.8 * path)
//	{
//		udp_send.send(id, vel[0] + step * accl[0], vel[1] + step * accl[1], 0, false, 100, 1);
//	}
//	while (distance <= 0.2*path)
//	{
//		udp_send.send(id, 0, 0, 0, false, 100, 1);
//	}
//}
//void speed_control(int id, udp_commu_send udp_send, int curx, int cury, int startx, int starty, int goalx, int goaly)
//{
//	int step = 5;
//	double distance = (curx - goalx)*(curx - goalx) + (cury - goaly)*(cury - goaly);
//	double path = (curx - startx)*(curx - startx) + (cury - starty)*(cury - starty);
//	if (distance <= 0.2 * path)
//	{
//		for (int i = 0; i < step; i++)
//		{
//			udp_send.send()
//		}
//	}
//}

void get_map(udp_commu_recv udp_recv)
{
	udp_recv.Recv(1);//1

	for (int i = 0; i < 450; i++)
	{
		for (int j = 0; j < 600; j++)
		{
			map_cost[i][j] = 0;
		}
	}
	number_map_blue_obstacles = udp_recv.vision_recv.robots_blue_size();
	for (int i = 0; i < udp_recv.vision_recv.robots_blue_size(); i++)
	{
		//udp_recv.Recv();
		if (udp_recv.robot_blue[i].id != test_car_number)
		{

			int x = floor(udp_recv.robot_blue[i].x * resolution);
			int y = floor(udp_recv.robot_blue[i].y * resolution);
			map_blue_obstacles_x[i] = x + 300;
			map_blue_obstacles_y[i] = y + 225;
			//cout << "map_blue_obstacles_x is " << map_blue_obstacles_x[i] << endl;
			//cout << "map_blue_obstacles_y is " << map_blue_obstacles_y[i] << endl;
			//cout << x << " " << y << endl;
			for (int i = int(x - car_radius * resolution - expand); i < int(x + car_radius * resolution + expand); i++)
			{
				for (int j = int(y - car_radius * resolution - expand); j < int(y + car_radius * resolution + expand); j++)
				{
					//map_cost[j + 225][i + 300] = 1;
				}
			}
		}
		//map_blue_obstacles_x[i] = -1;
		//map_blue_obstacles_y[i] = -1;
	}
	for (int i = number_map_blue_obstacles; i < 8; i++)
	{
		map_blue_obstacles_x[i] = -1;
		map_blue_obstacles_y[i] = -1;
	}

	number_map_yellow_obstacles = udp_recv.vision_recv.robots_yellow_size();
	for (int i = 0; i < udp_recv.vision_recv.robots_yellow_size(); i++)
	{

		//udp_recv.Recv();
		int x = floor(udp_recv.robot_yellow[i].x * resolution);
		int y = floor(udp_recv.robot_yellow[i].y * resolution);
		map_yellow_obstacles_x[i] = x + 300;
		map_yellow_obstacles_y[i] = y + 225;
		//cout << "map_yellow_obstacles_x is " << map_yellow_obstacles_x[i] << endl;
		//cout << "map_yellow_obstacles_y is " << map_yellow_obstacles_y[i] << endl;
		for (int i = int(x - car_radius * resolution - expand); i < int(x + car_radius * resolution + expand); i++)
		{
			for (int j = int(y - car_radius * resolution - expand); j < int(y + car_radius * resolution + expand); j++)
			{
				//map_cost[j + 225][i + 300] = 1;
			}
		}
	}
	for (int i = number_map_yellow_obstacles; i < 8; i++)
	{
		map_yellow_obstacles_x[i] = -1;
		map_yellow_obstacles_y[i] = -1;
	}

}

//ofstream saveFile;
//saveFile.open("mygrade.txt", ios::out | ios::trunc);
//int count = 0;
//for (int i = 0; i < 450; i++)
//{
//	for (int j = 0; j < 600; j++)
//	{
//		saveFile << map_cost[i][j];
//		count++;
//		if (count % 600 == 0)
//		{
//			saveFile << endl;
//		}
//	}
//}
//saveFile << endl;
//saveFile.close();

int fail_time = 1;
int if_fail = 0;

int main() {
	/*********************************************************************/
	udp_commu_recv udp_recv;
	udp_recv.my_id = test_car_number;
	udp_recv.Recv(1);
	udp_commu_send udp_send;
	udp_send.init();


	int pointnumber_smooth = 0;
	int x_path_smooth[600] = { 0 };
	int y_path_smooth[450] = { 0 };
	int* tempx;
	int* tempy;
	int flag = 0;
	int x = int(udp_recv.my_robot.x);
	int y = int(udp_recv.my_robot.y);
	cout << x << " " << y << endl;
	int goal_x = 50;
	int goal_y = 375;
	int start_x = 550;
	int start_y = 75;
	int temp_x;
	int temp_y;
	int nearest;

	udp_send.send(test_car_number, 0, vel_y, 0, false, )

	system("pause");
	return 0;
}

