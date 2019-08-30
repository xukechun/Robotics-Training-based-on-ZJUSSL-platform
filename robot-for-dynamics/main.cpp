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
double lethal_distance = 40.0;
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
int test_car_number = 2;
int number_map_blue_obstacles = 0;
int number_map_yellow_obstacles = 0;

//判断是否到达目标点的参数
double offset = 7;

//绘制路径
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

//根据不同情况设置不同膨胀半径
void set_expand(Debug_Msgs arc_set, udp_commu_recv udp_recv)
{
	udp_recv.Recv();
	int number = udp_recv.vision_recv.robots_blue + udp_recv.vision_recv.robots_yellow - 1;
	double* robot_x = new double[number];
	double* robot_y = new double[number];
	for (int i = 0; i < udp_recv.vision_recv.robots_blue; i++)
	{
		if (udp_recv.robot_blue[i].id != test_car_number)
		{
			robot_x[i] = udp_recv.vision_recv.robots_blue[i].x;
			robot_y[i] = udp_recv.vision_recv.robots_blue[i].y;
		}
	}
	for (int i = 0; i < udp_recv.vision_recv.robots_yellow; i++)
	{
		robot_x[i + udp_recv.vision_recv.robots_blue - 1] = udp_recv.vision_recv.robots_yellow[i].x;
		robot_y[i + udp_recv.vision_recv.robots_blue - 1] = udp_recv.vision_recv.robots_yellow[i].y;
	}

	for (int i = 0; i < number; i++)
	{
		Debug_Msg* line_set = new Debug_Msg();
		line_set = arc_set.add_msgs();
		line_set->set_type(Debug_Msg_Debug_Type_ARC);
		line_set->set_color(Debug_Msg_Color_GREEN);
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

//获取直线的斜率
double Get_angle(int x1, int y1, int x2, int y2) {
	double angle = (y2 - y1) / (x2 - x1);
	return angle;
}

//获取两点之间的距离
int Get_distance(int x1, int y1, int x2, int y2) {
	int distance = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	return distance;
}

//获取点到直线的距离
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

//设置排斥速度
int* run_away_velocity(double cur_x, double cur_y, double orientation) {
	//printf("cur_x %lf cur_y %lf\n", cur_x, cur_y);
	int obstacles_number = number_map_blue_obstacles + number_map_yellow_obstacles;
	int* run_away_velocity = new int[2];
	double run_velocity = 0;
	double repulsion = 200000000;
	double max_repulsion_v = 550;
	double distance = 1000;
	double run_away_angle = 0;
	run_away_velocity[0] = 0;
	run_away_velocity[1] = 0;
	//	cout << "run_away_velocity[0] " << run_away_velocity[0] << " run_away_velocity[1]" << run_away_velocity[1] << endl;
	//	cout << "number_map_blue_obstacles in rrt" << number_map_blue_obstacles << endl;
	//	cout << "number_map_yellow_obstacles in rrt" << number_map_yellow_obstacles << endl;
		//cout << "map_blue_obstacles_x 0 " << map_blue_obstacles_x[0] << "map_blue_obstacles_y 0 " << map_blue_obstacles_y[0] << endl;
	for (int i = 0; i < number_map_blue_obstacles; i++) {
		distance = sqrt(pow(map_blue_obstacles_x[i] - cur_x, 2) + pow(map_blue_obstacles_y[i] - cur_y, 2));
		//cout << "map_blue_obstacles_x " << map_blue_obstacles_x[i] << "map_blue_obstacles_y " << map_blue_obstacles_y[i] << endl;
//	distance = sqrt(pow(map_blue_obstacles_x[i] - cur_x, 2) + pow(map_blue_obstacles_y[i] - cur_y, 2)) - 12;
//	if (distance <= 60)
//	{
//			run_velocity = repulsion*(1/distance - 1/60)*1/(distance*distance);
////			cout << "run_velocity * cos(run_away_angle)" << run_velocity * cos(run_away_angle) << "run_velocity * sin(run_away_angle)" << run_velocity * sin(run_away_angle) << endl;
//			run_away_velocity[0] += (int)run_velocity * cos(run_away_angle);
//			run_away_velocity[1] += (int)run_velocity * sin(run_away_angle);
//	}

	}

	//	cout << "run_away_velocity[0] " << run_away_velocity[0] << " run_away_velocity[1]" << run_away_velocity[1] << endl;

	for (int i = 0; i < number_map_yellow_obstacles; i++) {
		distance = sqrt(pow(map_yellow_obstacles_x[i] - cur_x, 2) + pow(map_yellow_obstacles_y[i] - cur_y, 2));
		//cout << "map_yellow_obstacles_x " << map_yellow_obstacles_x[i] << "map_yellow_obstacles_y " << map_yellow_obstacles_y[i] << endl;
	//	cout << "yellow "<<i<<" " << distance;
		if (distance <= 60)
		{
			run_away_angle = atan2(cur_y - map_yellow_obstacles_y[i], cur_x - map_yellow_obstacles_x[i]);
			//run_velocity = repulsion*(1/distance - 1/80)*1/(distance*distance);
			if (0 < distance < 30)
			{
				run_velocity = 300;
			}
			else if (30 <= distance < 50)
			{
				run_velocity = 130;
			}
			else if (distance >= 50)
			{
				run_velocity = 100;
			}
			//			cout << "run_velocity * cos(run_away_angle)" << run_velocity * cos(run_away_angle) << "run_velocity * sin(run_away_angle)" << run_velocity * sin(run_away_angle) << endl;
			run_away_velocity[0] += (int)run_velocity * cos(run_away_angle - orientation);
			run_away_velocity[1] += (int)run_velocity * sin(run_away_angle - orientation);
		}
		//if (30 <= distance < 70) {
		//	run_away_angle = atan2(cur_y - map_blue_obstacles_y[i], cur_x - map_blue_obstacles_x[i]);
		//	run_velocity = repulsion / (distance * distance);
		//	//			cout << "run_velocity * cos(run_away_angle)" << run_velocity * cos(run_away_angle) << "run_velocity * sin(run_away_angle)" << run_velocity * sin(run_away_angle) << endl;
		//	run_away_velocity[0] += (int)run_velocity * cos(run_away_angle);
		//	run_away_velocity[1] += (int)run_velocity * sin(run_away_angle);

		//}
		//else if (0 < distance < 30)
		//{
		//	run_away_angle = atan2(cur_y - map_blue_obstacles_y[i], cur_x - map_blue_obstacles_x[i]);
		//	run_velocity = max_repulsion_v;
		//	//			cout << "run_velocity * cos(run_away_angle)" << run_velocity * cos(run_away_angle) << "run_velocity * sin(run_away_angle)" << run_velocity * sin(run_away_angle) << endl;
		//	run_away_velocity[0] += (int)run_velocity * cos(run_away_angle);
		//	run_away_velocity[1] += (int)run_velocity * sin(run_away_angle);
		//}

	}
	printf("run_away_vel[0] origin %d, run_away_vel[1] origin %d\n", run_away_velocity[0], run_away_velocity[1]);

	//	double danger_distance = 20;
	//	run_away_velocity = repulsion*(40-distance)/((distance-16)*(distance-16));
	return run_away_velocity;
}

//朝向目标速度
int* run_towards_velocity(int cur_x, int cur_y, int next_x, int next_y) {
	int* run_towards_velocity = new int[2];
	run_towards_velocity[0] = 0;
	run_towards_velocity[1] = 0;
	double attraction = 0.3;
	//	double run_towards_velocity[2] = { 0 };
	double angle = atan2(next_y - cur_y, next_x - cur_x);
	//	cout << "towards_angle " << angle << endl;
	double distance = sqrt(pow(cur_x - next_x, 2) + pow(cur_y - next_y, 2));
	//	cout << "towards_distance" << distance << endl;
	run_towards_velocity[0] = int(attraction * distance*cos(angle));
	run_towards_velocity[1] = int(attraction * distance*sin(angle));
	//	cout << "run_towards_vel[0] origin " << run_towards_velocity[0] << "run_towards_vel[1] origin " << run_towards_velocity[1] << endl;

	return run_towards_velocity;
}

//设置边界排斥速度
double * border_repulse(double cur_x, double cur_y, double ori) {
	double border_velocity_x = 0;
	double border_velocity_y = 0;
	double border_velocity_angle;
	double * border_velocity = new double[2];
	double vel[2];
	//	double border_vx = 0;
	//	double border_vy = 0;
	border_velocity[0] = 0;
	border_velocity[1] = 0;

	if ((cur_y < 50) || (cur_y > 400) || (cur_x < 50) || (cur_x > 550))
	{
		if (cur_y < 50) {
			border_velocity_y = (200 - cur_y)*(200 - cur_y) / 200;
			border_velocity_x = 0;
			//		border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
			border_velocity[0] += border_velocity_x;
			border_velocity[1] += border_velocity_y;
			//				cout << "y_1<50 " << "border_vx " << border_vx << "border_vy " << border_vy << endl;
		}

		if (cur_y > 400) {
			border_velocity_y = -(cur_y - 200)*(cur_y - 200) / 200;
			border_velocity_x = 0;
			//			border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
			border_velocity[0] += border_velocity_x;
			border_velocity[1] += border_velocity_y;
			//				cout << "y_1>400 " << "border_vx " << border_vx << "border_vy " << border_vy << endl;
		}

		if (cur_x < 50) {
			border_velocity_y = 0;
			border_velocity_x = (250 - cur_x)*(250 - cur_x) / 200;
			//			border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
			border_velocity[0] += border_velocity_x;
			border_velocity[1] += border_velocity_y;
			//				cout << "x_1<50 " << "border_vx " << border_vx << "border_vy " << border_vy << endl;

		}
		if (cur_x > 550) {
			border_velocity_y = 0;
			border_velocity_x = -(cur_x - 350)*(cur_x - 350) / 200;
			border_velocity[0] += border_velocity_x;
			border_velocity[1] += border_velocity_y;
			//				cout << "x_1>550 " << "border_vx " << border_vx << "border_vy " << border_vy << endl;

		}
	}
	vel[0] = border_velocity[0] * cos(ori) + border_velocity[1] * sin(ori);
	vel[1] = border_velocity[0] * sin(-ori) + border_velocity[1] * cos(ori);
	return vel;
}

//速度控制
double* setv(double cur_x, double cur_y, int start_x, int start_y, int goal_x, int goal_y, double oritation, int flag, double nearest)
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
			speed = sqrt(60 * temp_dis) + 50;
			if (temp_dis > 100)
				speed = 250;//250
		}
	}
	if (flag == 0) {
		if (nearest > 100) {
			if (temp_dis > distance / 2) {
				
				speed = 250;//350
			}
			else {
				speed = sqrt(60 * temp_dis) + 50;//+150
				if (temp_dis > 100)
					speed = 250;//250
			}
		}
		else if (nearest > 50) {
			speed = 50 + sqrt(100 * (nearest - 40));
		}
		else
			speed = 40;
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
	if (flag == 3) {
		if (nearest > 30)
			speed = sqrt(100 * (nearest - 30)) + 30;
		else
			speed = 20;
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

//获取地图
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


int fail_time = 1;
int if_fail = 0;

int main() {
	/*********************************************************************/
	//建立UDP对象和初始化
	udp_commu_recv udp_recv;
	udp_recv.my_id = test_car_number;
	udp_recv.Recv(1);
	udp_commu_send udp_send;
	udp_send.init();

	//参数初始化
	int pointnumber_smooth = 0;
	int x_path_smooth[600] = { 0 };
	int y_path_smooth[450] = { 0 };
	int* tempx;
	int* tempy;
	int flag = 0;
	int x = int(udp_recv.my_robot.x);
	int y = int(udp_recv.my_robot.y);
	//printf("%d %d\n", x, y);
	int goal_x = 50;
	int goal_y = 375;
	int start_x = 550;
	int start_y = 75;
	int temp_x;
	int temp_y;
	int nearest;


	RrtPath * rrtpath = new RrtPath;
	while (1)
	{
		//对于RRT规划失败次数太多的情况下缩小障碍物的膨胀半径
		if (fail_time == 0) {
			if_fail = 1;
			expand = 3;
			(*rrtpath).lethal_distance = 15.0;
			(*rrtpath).step = 40.0;
			fail_time = 1;
		}
		else {
			expand = EXPAND;
			(*rrtpath).lethal_distance = 25;
			(*rrtpath).step = 15.0;
		}
		//udp_recv.Recv(1);
		udp_recv.Recv();
		get_map(udp_recv);

		int x = int(udp_recv.my_robot.x);
		int y = int(udp_recv.my_robot.y);
		//cout << x << " " << endl;
		
		//RRT参数初始化
		pointnumber_smooth = 0;
		(*rrtpath).costs = map_cost;
		(*rrtpath).test_car_number = test_car_number;
		(*rrtpath).number_map_blue_obstacles = number_map_blue_obstacles;
		(*rrtpath).number_map_yellow_obstacles = number_map_yellow_obstacles;
		(*rrtpath).map_blue_obstacles_x = map_blue_obstacles_x;

		//			cout << "(*rrtpath).map_blue_obstacles_x[1] "<<(*rrtpath).map_blue_obstacles_x[1] << endl;
		(*rrtpath).map_blue_obstacles_y = map_blue_obstacles_y;
		//			cout << "(*rrtpath).map_blue_obstacles_x[1] " << (*rrtpath).map_blue_obstacles_x[1] << endl;
		(*rrtpath).map_yellow_obstacles_x = map_yellow_obstacles_x;
		(*rrtpath).map_yellow_obstacles_y = map_yellow_obstacles_y;

		//从起点到终点为flag = 0,终点到起点是flag = 1
		if (flag == 0) {
			temp_x = goal_x;
			temp_y = goal_y;
		}
		if (flag == 1) {
			temp_x = start_x;
			temp_y = start_y;
		}


		clock_t begintime, endtime;
		begintime = clock();
		if ((*rrtpath).getPath(x, y, temp_x, temp_y)) {
			endtime = clock();
			printf("path planning success!\n");
			//printf("plan time!!!!!!!!!!!!= %lf \n", double(endtime - begintime) / CLOCKS_PER_SEC);

			pointnumber_smooth = (*rrtpath).get_pointnumber_smooth();
			tempx = (*rrtpath).get_x_path_smooth();
			tempy = (*rrtpath).get_y_path_smooth();

			for (int i = 0; i < pointnumber_smooth; i++) {
				x_path_smooth[i] = tempx[i];
				y_path_smooth[i] = tempy[i];
				//cout << x_path_smooth[i] << " - " << y_path_smooth[i] << endl;
			}
		}
		else
		{
			fail_time = (fail_time + 1) % 20;
			printf("compute fail\n");
			continue;
		}

		//if (rrtpath.getPath(x, y, 600, 450, path)) cout << "point number of smooth path is  " << pointnumber_smooth << endl;
		////cout << "pointnumber" << pointnumber_smooth << endl;
		//vector<pair<int, int> > ::iterator iter;

		/********************************************************************************/

		Debug_Msgs lines_set;
		//lines_set.Clear();
		//clock_t begintime, endtime;
		begintime = clock();
		setpath(lines_set, x_path_smooth, y_path_smooth, pointnumber_smooth - 1);
		endtime = clock();

		//cout << "debug time!!!!!!!!!=" << double(endtime - begintime) / CLOCKS_PER_SEC << endl;

		/*********************************************************************************/

		//udp_recv.Recv(1);
		//udp_recv.Recv();

		int i = 0;
		//cout << "=" << pointnumber_smooth << endl;
		while (i < pointnumber_smooth - 1)
		{
			get_map(udp_recv);
			(*rrtpath).costs = map_cost;
			(*rrtpath).test_car_number = test_car_number;
			(*rrtpath).number_map_blue_obstacles = number_map_blue_obstacles;
			(*rrtpath).number_map_yellow_obstacles = number_map_yellow_obstacles;
			(*rrtpath).map_blue_obstacles_x = map_blue_obstacles_x;

			//			cout << "(*rrtpath).map_blue_obstacles_x[1] "<<(*rrtpath).map_blue_obstacles_x[1] << endl;
			(*rrtpath).map_blue_obstacles_y = map_blue_obstacles_y;
			//			cout << "(*rrtpath).map_blue_obstacles_x[1] " << (*rrtpath).map_blue_obstacles_x[1] << endl;
			(*rrtpath).map_yellow_obstacles_x = map_yellow_obstacles_x;
			(*rrtpath).map_yellow_obstacles_y = map_yellow_obstacles_y;
			(*rrtpath).lethal_distance = 20.0;
			//nearest = rrtpath->get_nearest_distance;

			udp_recv.Recv();
			double x_1, y_1;
			double* vel;
			double* repulse_vel;
			int dx = x_path_smooth[i + 1] - x_path_smooth[i];
			int dy = y_path_smooth[i + 1] - y_path_smooth[i];
			double alpha = atan2(dy, dx);
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			double orientation = udp_recv.my_robot.orientation;
			double temp_dis;
			//	orientation = udp_recv.my_robot.orientation;
			temp_dis = rrtpath->get_nearest_distance(x_1, y_1);
			vel = setv(x_1, y_1, x_path_smooth[i], y_path_smooth[i], x_path_smooth[i + 1], y_path_smooth[i + 1], orientation, 0, temp_dis);
			repulse_vel = border_repulse(x_1, y_1, orientation);
			//cout << vel[0] <<"--"<< vel[1] << endl;
			//udp_send.send(test_car_number, vel[0] + repulse_vel[0], vel[1] + repulse_vel[1], 0, false, 0, orientation - alpha);
			udp_send.send(test_car_number, vel[0], vel[1], 0, false, 0, orientation - alpha);

			udp_recv.Recv();
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			double dot_x = x_path_smooth[pointnumber_smooth - 1] - x_1;
			double dot_y = y_path_smooth[pointnumber_smooth - 1] - y_1;

			//如果当前位置与下一个节点之间存在障碍物，则重新规划
			if (if_fail = 1)
			{
				(*rrtpath).lethal_distance = 10;
				if (rrtpath->check_if_on_obstacle(x_1, y_1, x_path_smooth[i + 1], y_path_smooth[i + 1])) {
					udp_send.send(test_car_number, 0, 0, 0, false, 0, 0);
					printf("有障碍！重新规划rrt！\n");
					break;
				}
				if_fail = 0;
			}
			else {
				(*rrtpath).lethal_distance = 25;
				if (rrtpath->check_if_on_obstacle(x_1, y_1, x_path_smooth[i + 1], y_path_smooth[i + 1])) {
					udp_send.send(test_car_number, 0, 0, 0, false, 0, 0);
					printf("有障碍！重新规划rrt！\n");
					break;
				}
			}
			udp_recv.Recv();
			temp_dis = rrtpath->get_nearest_distance(x_1, y_1);
			
			//进入运动障碍物检测的判断，直到进入安全区才重新规划RRT
			if (temp_dis <= 60)
			{
				while (temp_dis <= 60) {
					get_map(udp_recv);
					int *runaway_velocity;
					udp_recv.Recv();
					orientation = udp_recv.my_robot.orientation;

					x_1 = udp_recv.my_robot.x;
					y_1 = udp_recv.my_robot.y;
					runaway_velocity = run_away_velocity(x_1, y_1, orientation);
					repulse_vel = border_repulse(x_1, y_1, orientation);
					temp_dis = rrtpath->get_nearest_distance(x_1, y_1);

					vel = setv(x_1, y_1, x_path_smooth[i], y_path_smooth[i], x_path_smooth[i + 1], y_path_smooth[i + 1], orientation, 0, temp_dis);
					//runaway_velocity[0] = runaway_velocity[0] + vel[0] + repulse_vel[0];
					//runaway_velocity[1] = runaway_velocity[1] + vel[1] + repulse_vel[1];
					runaway_velocity[0] = runaway_velocity[0] + vel[0];
					runaway_velocity[1] = runaway_velocity[1] + vel[1];
					//runaway_velocity[0] = runaway_velocity[0];
					//runaway_velocity[1] = runaway_velocity[1];

					udp_recv.Recv();
					int * runtowards_velocity;
					double	vel_composed_angle = atan2(runaway_velocity[1], runaway_velocity[0]);

					udp_send.send(test_car_number, runaway_velocity[0], runaway_velocity[1], 0, false, 0, 0);

					udp_recv.Recv();
					x_1 = udp_recv.my_robot.x;
					y_1 = udp_recv.my_robot.y;
					temp_dis = rrtpath->get_nearest_distance(x_1, y_1);

				}
				printf("附近有障碍物，避障！重新规划！\n");
				udp_send.send(test_car_number, 0, 0, 0, false, 0, 0);
				break;
			}

			if (i == pointnumber_smooth - 2) {
				if (Get_distance(x_1, y_1, start_x, start_y) <= offset && flag == 1)
				{
					flag = 0;
					printf("到达起点！重新规划！\n");
					break;
				}
				if (Get_distance(x_1, y_1, goal_x, goal_y) <= offset && flag == 0)
				{
					flag = 1;
					printf("到达终点！重新规划！\n");
					break;
				}
				udp_recv.Recv();
				x_1 = udp_recv.my_robot.x;
				y_1 = udp_recv.my_robot.y;
				orientation = udp_recv.my_robot.orientation;
				repulse_vel = border_repulse(x_1, y_1, orientation);
				vel = setv(x_1, y_1, x_path_smooth[i], y_path_smooth[i], x_path_smooth[i + 1], y_path_smooth[i + 1], orientation, 1, 0);
				//udp_send.send(test_car_number, vel[0] + repulse_vel[0], vel[1] + repulse_vel[1], 0, false, 0, 0);
				udp_send.send(test_car_number, vel[0], vel[1], 0, false, 0, 0);
				continue;
			}
			if (sqrt((x_path_smooth[i + 1] - x_1)*(x_path_smooth[i + 1] - x_1) + (y_path_smooth[i + 1] - y_1)*(y_path_smooth[i + 1] - y_1)) <= 9) {
				i++;
				continue;
			}

			delete vel;
		}
	}
	delete rrtpath;
	system("pause");
	return 0;
}

