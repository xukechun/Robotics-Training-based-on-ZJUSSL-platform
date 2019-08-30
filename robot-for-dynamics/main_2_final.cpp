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
	printf("%d %d\n", x, y);
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
		if (fail_time == 0) {
			if_fail = 1;
			expand = 3;
			(*rrtpath).lethal_distance = 15.0;
			(*rrtpath).step = 40.0;
			fail_time = 1;
		}
		else {
			expand = EXPAND;
			(*rrtpath).lethal_distance = 40;
			(*rrtpath).step = 15.0;
		}
		//udp_recv.Recv(1);
		udp_recv.Recv();
		get_map(udp_recv);

		int x = int(udp_recv.my_robot.x);
		int y = int(udp_recv.my_robot.y);
		//cout << x << " " << endl;

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


		if (flag == 0) {
			temp_x = goal_x;
			temp_y = goal_y;
		}
		if (flag == 1) {
			temp_x = start_x;
			temp_y = start_y;
		}

		udp_recv.Recv();
		clock_t begintime, endtime;
		begintime = clock();
		if ((*rrtpath).getPath(x, y, temp_x, temp_y)) {
			endtime = clock();
			printf("path planning success!\n");
			printf("plan time!!!!!!!!!!!!= %lf \n", double(endtime - begintime) / CLOCKS_PER_SEC);

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
			int dx = x_path_smooth[i + 1] - x_path_smooth[i];
			int dy = y_path_smooth[i + 1] - y_path_smooth[i];
			double alpha = atan2(dy, dx);
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			double orientation = udp_recv.my_robot.orientation;

			//	orientation = udp_recv.my_robot.orientation;

			vel = setv(x_1, y_1, x_path_smooth[i], y_path_smooth[i], x_path_smooth[i + 1], y_path_smooth[i + 1], orientation, 0);
			//cout << vel[0] <<"--"<< vel[1] << endl;
			udp_send.send(test_car_number, vel[0], vel[1], 0, false, 0, orientation - alpha);
			//double vel_x = udp_recv.my_robot.raw_vel_x  * cos(udp_recv.my_robot.orientation);
			//double vel_y = udp_recv.my_robot.raw_vel_y  * sin(udp_recv.my_robot.orientation);

			//cout << "raw velocity now: " << sqrt(vel_x*vel_x + vel_y * vel_y) << endl;


			//cout << x_1 << " " << y_1 << endl;
			/*if (flag == 1) {
				cout << "(start_x - x_1)*(start_x - x_1) + (start_y - y_1)*(start_y - y_1) = " << (start_x - x_1)*(start_x - x_1) + (start_y - y_1)*(start_y - y_1) << endl;

			}
			if (flag == 0) {
				cout << "(goal_x - x_1)*(goal_x - x_1) + (goal_y - y_1)*(goal_y - y_1) = " << (goal_x - x_1)*(goal_x - x_1) + (goal_y - y_1)*(goal_y - y_1) << endl;

			}*/
			udp_recv.Recv();
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			double dot_x = x_path_smooth[pointnumber_smooth - 1] - x_1;
			double dot_y = y_path_smooth[pointnumber_smooth - 1] - y_1;

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
			double temp_dis = rrtpath->get_nearest_distance(x_1, y_1);

			if (temp_dis <= 60) {
				double *get_nearest_point = (*rrtpath).get_nearest_point(x_1, y_1);
				double run_away_orientation[2];
				run_away_orientation[0] = x_1 - get_nearest_point[0];
				run_away_orientation[1] = y_1 - get_nearest_point[1];
				double run_away_orientation_angle = atan2(run_away_orientation[1], run_away_orientation[0]);
				//				double nearest_distance = rrtpath->get_nearest_distance(x_1, y_1);
				double run_velocity;
				run_velocity = run_away_velocity(x_1, y_1, get_nearest_point[0], get_nearest_point[1]);
				double *vel_revise;
				vel_revise[0] = vel[0] + run_velocity * cos(run_away_orientation_angle - orientation);
				vel_revise[1] = vel[1] + run_velocity * sin(run_away_orientation_angle - orientation);
				//	double *oba;
				//	oba = rrtpath->get_nearest_point(x_1, y_1);
					//					setv(x_1, y_1, oba[0], oba[1], 0, 0, orientation, 2);
				udp_send.send(test_car_number, vel_revise[0], vel_revise[1], 0, false, 0, orientation - alpha);
				printf("附近有障碍物，避障！重新规划！\n");
				//double vel_x = udp_recv.my_robot.raw_vel_x  * cos(udp_recv.my_robot.orientation);
				//double vel_y = udp_recv.my_robot.raw_vel_y  * sin(udp_recv.my_robot.orientation);

			//	cout << "run_away velocity: " << sqrt(vel_x*vel_x + vel_y * vel_y) << endl;

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
				orientation = udp_recv.my_robot.orientation;
				vel = setv(x_1, y_1, x_path_smooth[i], y_path_smooth[i], x_path_smooth[i + 1], y_path_smooth[i + 1], orientation, 1);
				udp_send.send(test_car_number, vel[0], vel[1], 0, false, 0, orientation - alpha);
				continue;
			}
			if (sqrt((x_path_smooth[i + 1] - x_1)*(x_path_smooth[i + 1] - x_1) + (y_path_smooth[i + 1] - y_1)*(y_path_smooth[i + 1] - y_1)) <= 9) {
				i++;
				continue;
			}

			delete vel;
		}
		//i = pointnumber_smooth - 1;
		//while (i > 0)
		//{
		//	udp_recv.Recv();
		//	if (udp_recv.vision_recv.robots_blue_size() > 0) {
		//		udp_recv.Recv();
		//		int x, y;
		//		x = udp_recv.my_robot.x;
		//		y = udp_recv.my_robot.y;

		//		double orientation = udp_recv.my_robot.orientation;
		//		//cout << x_path_smooth[i - 1] << " * " << y_path_smooth[i - 1] << endl;
		//		double* vel = setv(x, y, x_path_smooth[i - 1], y_path_smooth[i - 1], orientation);
		//		udp_send.send(0, vel[0], vel[1], 0, false, 100, 1);
		//		udp_recv.Recv();
		//		/*if (rrtpath->check_if_on_obstacle(x, y, x_path_smooth[i + 1], y_path_smooth[i + 1])) {
		//			break;
		//		}*/
		//		if ((x_path_smooth[0] - x)*(x_path_smooth[0] - x) + (y_path_smooth[0] - y)*(y_path_smooth[0] - y) <= 4)
		//		{
		//			udp_send.send(0, 0, 0, 0, false, 0, 1);
		//		}

		//		if ((x_path_smooth[i - 1] - x)*(x_path_smooth[i - 1] - x) + (y_path_smooth[i - 1] - y)*(y_path_smooth[i - 1] - y) <= 4)
		//		{
		//			i--;
		//			continue;
		//		}
		//	}

		//}
	}
	delete rrtpath;
	system("pause");
	return 0;
}

