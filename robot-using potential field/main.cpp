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
#include <random>


int EXPAND = 20;
int Max_vel = 400;
int expand;
using namespace std;
#pragma comment(lib, "WS2_32.lib")


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



/*Rejection velocity given by obstacles*/
int* run_away_velocity(double cur_x, double cur_y, int flag) {
	int obstacles_number = number_map_blue_obstacles + number_map_yellow_obstacles;
	int* run_away_velocity = new int[2];
	double run_velocity = 0;
	double repulsion = 200000000;
	double max_repulsion_v = 550;
	double distance = 1000;
	double run_away_angle = 0;
	run_away_velocity[0] = 0;
	run_away_velocity[1] = 0;

	for (int i = 0; i < number_map_blue_obstacles; i++) {
		distance = sqrt(pow(map_blue_obstacles_x[i] - cur_x, 2) + pow(map_blue_obstacles_y[i] - cur_y, 2));

		for (int i = 0; i < number_map_yellow_obstacles; i++) {
			distance = sqrt(pow(map_yellow_obstacles_x[i] - cur_x, 2) + pow(map_yellow_obstacles_y[i] - cur_y, 2));
			//step function
			if (distance <= 60)
			{
				run_away_angle = atan2(cur_y - map_yellow_obstacles_y[i], cur_x - map_yellow_obstacles_x[i]);

				if (0 < distance < 30)
				{
					run_velocity = 150;
				}
				else if (30 <= distance < 50)
				{
					run_velocity = 100;
				}
				else if (distance >= 50)
				{
					run_velocity = 50;
				}
				run_away_velocity[0] += (int)run_velocity * cos(run_away_angle);
				run_away_velocity[1] += (int)run_velocity * sin(run_away_angle);
			}


		}
		printf("run_away_vel[0] origin %d, run_away_vel[1] origin %d\n", run_away_velocity[0], run_away_velocity[1]);




		/*guided velocity in different zones to avoid obstacles*/
		double distance_f = 0;
		if (flag == 0) {
			if (cur_x > 300 && cur_y < 320) run_away_velocity[1] += 150;
			if (cur_x > 300 && cur_y > 325)  run_away_velocity[0] -= 50;

			distance_f = sqrt((cur_x - 50) *(cur_x - 50) + (cur_y - 375) *(cur_y - 375));

		}
		if (flag == 1) {
			distance_f = sqrt((cur_x - 550) *(cur_x - 550) + (cur_y - 75) *(cur_y - 75));
			if (cur_x < 300 && cur_y > 180)
			{
				run_away_velocity[1] -= 150;
			}
			if (cur_x < 300 && cur_y < 125)
			{
				run_away_velocity[0] += 50;
			}
		}

		if (distance_f < 60) {
			run_away_velocity[1] = 0;
			run_away_velocity[0] = 0;
		}
		cout << "run_away_vel[0] origin " << run_away_velocity[0] << "run_away_vel[1] origin " << run_away_velocity[1] << endl;

		return run_away_velocity;

	}


	/*attraction velocity given by goals*/
	int* run_towards_velocity(int cur_x, int cur_y, int next_x, int next_y) {
		int* run_towards_velocity = new int[2];
		run_towards_velocity[0] = 0;
		run_towards_velocity[1] = 0;
		double attraction = 0.3;
		
		double angle = atan2(next_y - cur_y, next_x - cur_x);
		
		double distance = sqrt(pow(cur_x - next_x, 2) + pow(cur_y - next_y, 2));
		if (distance > 40) {
			run_towards_velocity[0] = int(attraction * (distance + 400)*cos(angle));
			run_towards_velocity[1] = int(attraction *(distance + 400)*sin(angle));
		}
		else {
			double speed = sqrt(60 * distance) + 40;
			run_towards_velocity[0] = int(speed*cos(angle));
			run_towards_velocity[1] = int(speed*sin(angle));

		}
		return run_towards_velocity;
	}


	/*get coordinates of the nearest point*/
	double*  get_nearest_point(double test_x, double test_y) {
		double nearest_distance;
		double nearest_point[2];
		if (number_map_blue_obstacles == 0 || number_map_yellow_obstacles == 0) {
			nearest_point[0] = nearest_point[1] = 0;
			return nearest_point;
		}
		else {
			if (number_map_blue_obstacles > 0) {
				nearest_distance = sqrt(pow(fabs(test_x - map_blue_obstacles_x[0]), 2) + pow(fabs(test_y - map_blue_obstacles_y[0]), 2));
				nearest_point[0] = map_blue_obstacles_x[0];
				nearest_point[1] = map_blue_obstacles_y[0];
			}
			else {
				nearest_distance = sqrt(pow(fabs(test_x - map_yellow_obstacles_x[0]), 2) + pow(fabs(test_y - map_yellow_obstacles_y[0]), 2));
				nearest_point[0] = map_yellow_obstacles_x[0];
				nearest_point[1] = map_yellow_obstacles_y[0];
			}
		}

		double now_distance = 0.0;
		for (int i = 0; i < number_map_blue_obstacles; i++) {
			now_distance = sqrt(pow(fabs(test_x - map_blue_obstacles_x[i]), 2) + pow(fabs(test_y - map_blue_obstacles_y[i]), 2));
			if (now_distance < nearest_distance) {
				nearest_distance = now_distance;
				nearest_point[0] = map_blue_obstacles_x[i];
				nearest_point[1] = map_blue_obstacles_y[i];
			}
		}
		for (int i = 0; i < number_map_yellow_obstacles; i++) {
			now_distance = sqrt(pow(fabs(test_x - map_yellow_obstacles_x[i]), 2) + pow(fabs(test_y - map_yellow_obstacles_y[i]), 2));
			if (now_distance < nearest_distance) {
				nearest_distance = now_distance;
				nearest_point[0] = map_yellow_obstacles_x[i];
				nearest_point[1] = map_yellow_obstacles_y[i];
			}
		}
		//cout << "nearest_distance is " << nearest_distance << endl;
		return nearest_point;
	}


	/*velocity control function*/
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
				speed = sqrt(800 * abs(distance - temp_dis)) + 50;//+50
				if (distance - temp_dis > 100)
					speed = 350;//350
			}
			else {
				speed = sqrt(60 * temp_dis) + 50;//+50
				if (temp_dis > 100)
					speed = 250;//250
			}
		}
		if (flag == 0) {
			if (temp_dis > distance / 2) {
				speed = sqrt(800 * abs(distance - temp_dis)) + 20;//+20
				if (distance - temp_dis > 100)
					speed = 350;//350
			}
			else {
				speed = sqrt(60 * temp_dis) + 50;//+50
				if (temp_dis > 100)
					speed = 20;//250
			}
		}
		if (flag == 2) {
			double dx_2 = cur_x - start_x;
			double dy_2 = cur_y - start_y;
			double temp_dis2 = Get_distance(cur_x, cur_y, start_x, start_y);
			double alpha_2 = atan2(dy_2, dx_2);

			speed = sqrt(60 * temp_dis);
			double speed_2 = sqrt(800 * abs(100 - temp_dis)) - 10;

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

		vel[0] = dv_x + temp_vx;
		vel[1] = dv_y + temp_vy;

		/*vel[0] = temp_vx;
		vel[1] = temp_vy;*/
		return vel;
	}


	/*get coordinates of obstacles*/
	void get_map(udp_commu_recv udp_recv)
	{
		udp_recv.Recv(1);

		for (int i = 0; i < 450; i++)
		{
			for (int j = 0; j < 600; j++)
			{
				map_cost[i][j] = 0;
			}
		}
		number_map_blue_obstacles = udp_recv.vision_recv.robots_blue_size() - 1;
		int blue_obs = 0;
		
		for (int i = 0; i < udp_recv.vision_recv.robots_blue_size(); i++)
		{
			if (udp_recv.robot_blue[i].id != test_car_number)
			{

				int x = floor(udp_recv.robot_blue[i].x * resolution);
				int y = floor(udp_recv.robot_blue[i].y * resolution);
				map_blue_obstacles_x[blue_obs] = x + 300;
				map_blue_obstacles_y[blue_obs] = y + 225;

				blue_obs++;
				//cout << x << " " << y << endl;
				for (int i = int(x - car_radius * resolution - expand); i < int(x + car_radius * resolution + expand); i++)
				{
					for (int j = int(y - car_radius * resolution - expand); j < int(y + car_radius * resolution + expand); j++)
					{
						//				map_cost[j + 225][i + 300] = 1;
					}
				}
			}

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

			for (int i = int(x - car_radius * resolution - expand); i < int(x + car_radius * resolution + expand); i++)
			{
				for (int j = int(y - car_radius * resolution - expand); j < int(y + car_radius * resolution + expand); j++)
				{
					//			map_cost[j + 225][i + 300] = 1;
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
	int if_need_turbu = 0;
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

		udp_recv.Recv();
		int startx_1 = int(udp_recv.my_robot.x);
		int starty_1 = int(udp_recv.my_robot.y);

		RrtPath * rrtpath = new RrtPath;
		while (1)
		{
			if (flag == 0) {
				temp_x = goal_x;
				temp_y = goal_y;
			}
			if (flag == 1) {
				temp_x = start_x;
				temp_y = start_y;
			}
			
			get_map(udp_recv);
			udp_recv.Recv();
			int x_1 = int(udp_recv.my_robot.x);
			int y_1 = int(udp_recv.my_robot.y);

			pointnumber_smooth = 0;

			/*get map information*/
			(*rrtpath).costs = map_cost;
			(*rrtpath).test_car_number = test_car_number;
			(*rrtpath).number_map_blue_obstacles = number_map_blue_obstacles;
			(*rrtpath).number_map_yellow_obstacles = number_map_yellow_obstacles;
			(*rrtpath).map_blue_obstacles_x = map_blue_obstacles_x;
			(*rrtpath).map_blue_obstacles_y = map_blue_obstacles_y;
			(*rrtpath).map_yellow_obstacles_x = map_yellow_obstacles_x;
			(*rrtpath).map_yellow_obstacles_y = map_yellow_obstacles_y;


			double orientation;

			udp_recv.Recv();
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;

			/*get rejection velocity*/
			int *runaway_velocity = new int;
			runaway_velocity = run_away_velocity(x_1, y_1, flag);
			//		cout << "runaway_velocity_0 " << runaway_velocity[0] << "runaway_velocity_1 " << runaway_velocity[1] << endl;
			double run_away_velocity = sqrt(runaway_velocity[0] * runaway_velocity[0] + runaway_velocity[1] * runaway_velocity[1]);
			double run_away_orientation = atan2(runaway_velocity[1], runaway_velocity[0]);


			/*get attraction velocity*/
			int * runtowards_velocity;
			runtowards_velocity = run_towards_velocity(x_1, y_1, temp_x, temp_y);
			double run_towards_velocity = sqrt(runtowards_velocity[0] * runtowards_velocity[0] + runtowards_velocity[1] * runtowards_velocity[1]);
			double run_towards_orientation = atan2(runtowards_velocity[1], runtowards_velocity[0]);

			/*send velocity to robot*/
			udp_recv.Recv();
			orientation = udp_recv.my_robot.orientation;
			double vel_composed[2];
			vel_composed[0] = run_away_velocity * cos(run_away_orientation - orientation) + run_towards_velocity * cos(run_towards_orientation - orientation);
			vel_composed[1] = run_away_velocity * sin(run_away_orientation - orientation) + run_towards_velocity * sin(run_towards_orientation - orientation);
			double	vel_composed_angle = atan2(vel_composed[1], vel_composed[0]);
			for (int i = 0; i < 5; i++) {
				udp_send.send(test_car_number, vel_composed[0], vel_composed[1], 0, false, 0, 0);
			}


			/*boundary repulsion*/
			udp_recv.Recv();
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			double border_velocity_x = 0;
			double border_velocity_y = 0;
			double border_velocity_angle;
			orientation = udp_recv.my_robot.orientation;
			double border_vx = 0;
			double border_vy = 0;
			if ((y_1 < 50) || (y_1 > 400) || (x_1 < 50) || (x_1 > 550))
			{
				//on the bottom
				if (y_1 < 50) {
					border_velocity_y = (200 - y_1)*(200 - y_1) / 25;
					border_velocity_x = 0;
					border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
					border_vx += 300 * cos(border_velocity_angle - orientation);
					border_vy += 300 * sin(border_velocity_angle - orientation);

				}

				//on the top
				if (y_1 > 400) {
					border_velocity_y = -(y_1 - 200)*(y_1 - 200) / 25;
					border_velocity_x = 0;
					border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
					border_vx += 300 * cos(border_velocity_angle - orientation);
					border_vy += 300 * sin(border_velocity_angle - orientation);

				}

				//in the left
				if (x_1 < 50) {
					border_velocity_y = 0;
					border_velocity_x = (250 - x_1)*(250 - x_1) / 25;
					border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
					border_vx += 300 * cos(border_velocity_angle - orientation);
					border_vy += 300 * sin(border_velocity_angle - orientation);

				}

				// in the right
				if (x_1 > 550) {
					border_velocity_y = 0;
					border_velocity_x = -(x_1 - 350)*(x_1 - 350) / 25;
					border_velocity_angle = atan2(border_velocity_y, border_velocity_x);
					border_vx += 300 * cos(border_velocity_angle - orientation);
					border_vy += 300 * sin(border_velocity_angle - orientation);
				}

				for (int i = 0; i < 3; i++) {
					udp_send.send(test_car_number, border_vx, border_vy, 0, false, 0, 0);
				}

			}


			/*give random disturbance when in a potential well*/
			if (my_v < 300) {

				if (!(((x_1 < 70) && (y_1 > 380)) || ((y_1 < 70) && (x_1 > 530)))) {
					if_need_turbu = 1;
					std::default_random_engine generator(time(NULL));
					std::uniform_int_distribution<int> random_num(0, 999);

					//generate random angle of the disturbance
					double rand_num = random_num(generator) / 1000.0;
					double rand_angle = 3.14159*(2 * rand_num - 1);

					cout << "rand_angle " << rand_angle << endl;
					int vel_x = 600 * cos(rand_angle - orientation);
					int vel_y = 600 * sin(rand_angle - orientation);
					for (int i = 0; i < 5; i++) {
						udp_send.send(test_car_number, vel_x, vel_y, 0, false, 0, 0);
					}
				}

			}

			udp_recv.Recv();

			/*goal and start point swap*/
			x_1 = udp_recv.my_robot.x;
			y_1 = udp_recv.my_robot.y;
			if (Get_distance(x_1, y_1, start_x, start_y) <= offset && flag == 1)
			{
				flag = 0;
				printf("到达起点！重新规划！\n");
				continue;
			}
			if (Get_distance(x_1, y_1, goal_x, goal_y) <= offset && flag == 0)
			{
				flag = 1;
				printf("到达终点！重新规划！\n");
				continue;
			}
			delete runaway_velocity;
		}

		system("pause");
		return 0;
	}

