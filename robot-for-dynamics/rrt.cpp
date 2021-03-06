#include "rrt.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <random>
#include <iostream>
#include <ctime>
#include <cassert>
#include <cstring>
#include <vector>
#include<time.h>

// #include "UDP.h"

using namespace std;
//namespace my_planner {

//int **costs;
//int N = 600 * 450;
//int **costs = (int**)malloc(N * sizeof(int*)) ;

RrtPath::RrtPath() {
}

RrtPath::~RrtPath() {
	///	pointnumber = 0;
	//	pointnumber_smooth = 0;
	//	memset(x_path, 0, 600 * sizeof(int));
	//	memset(y_path, 0, 450 * sizeof(int));
		//path_new.clear();
}
double RrtPath::get_nearest_distance(double test_x, double test_y) {
	double nearest_distance=1000;
	//if (number_map_blue_obstacles == 0 || number_map_yellow_obstacles == 0) {
	//	return 500;
	//}
	//else {
	//	if (number_map_blue_obstacles > 0) {
	//		nearest_distance = sqrt(pow(fabs(test_x - map_blue_obstacles_x[0]), 2) + pow(fabs(test_y - map_blue_obstacles_y[0]), 2));
	//	}
	//	else {
	//		nearest_distance = sqrt(pow(fabs(test_x - map_yellow_obstacles_x[0]), 2) + pow(fabs(test_y - map_yellow_obstacles_y[0]), 2));
	//	}
	//}

	double now_distance = 0.0;
	for (int i = 0; i < number_map_blue_obstacles; i++) {
		//	cout << "test_x" << test_x << "test_y" << test_y << endl;
		//	cout << "map_blue_obstacles_x[i] in get_nearest_distance is" << map_blue_obstacles_x[i] << endl;
		now_distance = sqrt(pow(fabs(test_x - map_blue_obstacles_x[i]), 2) + pow(fabs(test_y - map_blue_obstacles_y[i]), 2));
		//	cout << "now_distance" << now_distance << endl;
		if (now_distance < nearest_distance) {
			nearest_distance = now_distance;
		}
	}
	for (int i = 0; i < number_map_yellow_obstacles; i++) {
		now_distance = sqrt(pow(fabs(test_x - map_yellow_obstacles_x[i]), 2) + pow(fabs(test_y - map_yellow_obstacles_y[i]), 2));
		if (now_distance < nearest_distance) {
			nearest_distance = now_distance;

		}
	}
	//cout << "nearest_distance is " << nearest_distance << endl;
	return nearest_distance;
}

double* RrtPath::get_nearest_point(double test_x, double test_y) {
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
		//	cout << "test_x" << test_x << "test_y" << test_y << endl;
		//	cout << "map_blue_obstacles_x[i] in get_nearest_distance is" << map_blue_obstacles_x[i] << endl;
		now_distance = sqrt(pow(fabs(test_x - map_blue_obstacles_x[i]), 2) + pow(fabs(test_y - map_blue_obstacles_y[i]), 2));
		//	cout << "now_distance" << now_distance << endl;
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


bool  RrtPath::getPath(int start_x, int start_y, int end_x, int end_y)
{
	printf("start x is %d, start y is %d\n", start_x, start_y);
	//cout << "map_blue_obstacles_x in getPath is " << map_blue_obstacles_x[1] << endl;
	//cout << "map_blue_obstacles_y in getPath is " << map_blue_obstacles_y[1] << endl;
	//cout << "map_blue_obstacles_x in getPath is" << map_blue_obstacles_x[2] << endl;
	//cout << "map_blue_obstacles_y in getPath is " << map_blue_obstacles_y[2] << endl;
	for (int i = 0; i < 600; i++) {
		x_path[i] = 0;
		x_path_smooth[i] = 0;
	}
	for (int i = 0; i < 450; i++)
	{
		y_path[i] = 0;
		y_path_smooth[i] = 0;
	}
	int init_pointnumber = 0;
	pointnumber = 0;
	pointnumber_smooth = 0;
	ns_ = 600 * 450;
	//       std::fill(potential, potential + ns_, POT_HIGH);
	//       lethal_cost_ = 253; //待改

        //待改
	vector<pair<int, int>>  path;
	std::vector<point> seeds;
	point end_index;
	end_index.pos_x = end_x;
	end_index.pos_y = end_y;
	point current;
	current.pos_x = start_x;
	current.pos_y = start_y;
	current.parent = -1;
	seeds.push_back(current);
	init_pointnumber = 1;
	int cycles = 0;

	std::default_random_engine generator(time(NULL));
	int flag = 0;
	//循环数cycles待改
	while (cycles < 2000) {
		cycles++;
		//	cout << "here" << endl;
		std::uniform_int_distribution<int> random_gen_y(25, 425);//范围待改
		std::uniform_int_distribution<int> random_gen_x(20, 580);//待改
		int ran_x = random_gen_x(generator);
		int ran_y = random_gen_y(generator);
		current.pos_x = ran_x;
		current.pos_y = ran_y;
		int nearest = get_nearest(seeds, current);
		if (distance(seeds[nearest], current) < step) {
			cycles--;
			continue;
		}
		else {
			double delta_x_end = end_index.pos_x - seeds[nearest].pos_x;
			double delta_y_end = end_index.pos_y - seeds[nearest].pos_y;
			double delta_x = current.pos_x - seeds[nearest].pos_x;
			double delta_y = current.pos_y - seeds[nearest].pos_y;
			double rate = sqrt(pow(step, 2) / (pow(fabs(delta_x), 2) + pow(fabs(delta_y), 2)));
			double rate_end = sqrt(pow(step, 2) / (pow(fabs(delta_x_end), 2) + pow(fabs(delta_y_end), 2)));
			double increment_x = seeds[nearest].pos_x + delta_x * rate;
			double increment_y = seeds[nearest].pos_y + delta_y * rate;
			int increment_x1 = (int)increment_x;
			int increment_y1 = (int)increment_y;


			/*如果下一个点在障碍物，则重新选择*/
			if (get_nearest_distance(increment_x1, increment_y1) < lethal_distance || (check_if_on_obstacle(seeds[nearest].pos_x, seeds[nearest].pos_y, increment_x1, increment_y1)) || increment_x1 < 0 || increment_x1>620 || increment_y1 < 0 || increment_y1>470)
			{
				/*                 if (use_auto_path) {
										double _increment_x = seeds[nearest].pos_x + delta_x * rate * alpha;
										double _increment_y = seeds[nearest].pos_y + delta_y * rate * alpha;
										if (costs[getIndex(_increment_x, _increment_y)] > 0)
											continue;
										else {
											increment_x = _increment_x;
											increment_y = _increment_y;
										}
									}
									else*/
				continue;
			}

			current.pos_x = increment_x1;
			current.pos_y = increment_y1;
			current.parent = nearest;
			seeds.push_back(current);
			init_pointnumber++;

			//             potential[getIndex(current.pos_x, current.pos_y)] = 255;
			if (distance(end_index, current) < step)
			{
				flag = 1;
				break;
			}
		}
	}
	if (flag == 0)
	{
		printf("Path planning failed! \n");
		return false;
	}
	end_index.parent = seeds.size() - 1;

	std::pair<int, int> end_;
	std::pair<int, int> start_;
	int x_index = 0;
	int y_index = 0;
	end_.first = end_index.pos_x;
	end_.second = end_index.pos_y;
	start_.first = start_x;
	start_.second = start_y;
	path.push_back(end_);

	std::pair<int, int> now;

	while (current.parent != -1) {
		point temp1 = seeds[current.parent];
		if (temp1.parent != -1) {
			point temp2 = seeds[temp1.parent];
			if (distance(current, temp2) < (distance(current, temp1) + distance(temp1, temp2))) {
				if (!check_if_on_obstacle(current.pos_x, current.pos_y, temp2.pos_x, temp2.pos_y)) {
					now.first = current.pos_x;
					now.second = current.pos_y;
					path.push_back(now);
					current = seeds[temp1.parent];
					continue;
				}
			}
			now.first = current.pos_x;
			now.second = current.pos_y;
			path.push_back(now);
			current = seeds[current.parent];
			continue;
		}
		now.first = current.pos_x;
		now.second = current.pos_y;
		path.push_back(now);
		current = seeds[current.parent];
		break;
	}

	path.push_back(start_);

	vector<pair<int, int> > ::reverse_iterator iter;
	printf("开始RRT\n");
	int count = 0;
	for (iter = path.rbegin(); iter != path.rend(); iter++) {
	//	cout << (*iter).first << " && "<<(*iter).second << endl;

		x_path[pointnumber] = (*iter).first;
		y_path[pointnumber] = (*iter).second;
		pointnumber++;
	}
	printf("pointnumber is %d\n", pointnumber);
	
	//x_path[x_index] = start_x;
	//y_path[y_index] = start_y;
	//cout << pointnumber << " # " << endl;
	printf("start (%d, %d) \n", x_path[0], y_path[0]);
	printf("end (%d, %d) \n", x_path[pointnumber - 1], y_path[pointnumber - 1]);
	//point test1;
	//point test2;
	//test1.pos_x = 406;
	//test1.pos_y = 174;

	//test2.pos_x = 10;
	//test2.pos_y = 10;
	//cout << check_if_on_obstacle(test1, test2) << endl;
	int start = 0;
	int end = pointnumber - 1;
	x_path_smooth[0] = x_path[0];
	y_path_smooth[0] = y_path[0];
	pointnumber_smooth = 1;
	cout << x_path_smooth[0] << " - - " << y_path_smooth[0] << endl;

	if (pointnumber == 2) {
		pointnumber_smooth = 2;
		x_path_smooth[0] = x_path[0];
		y_path_smooth[0] = y_path[0];
		x_path_smooth[1] = x_path[1];
		y_path_smooth[1] = y_path[1];
		printf("Path planning success!\n");
	
		return true;
	}
	else
	{
		for (int i = 0; i < pointnumber && start < pointnumber - 1; i++) {
			 end = pointnumber - 1;
			int start_pos_x = x_path[start];
			int start_pos_y = y_path[start];
			int end_pos_x = x_path[end];
			int end_pos_y = y_path[end];
			while (check_if_on_obstacle(start_pos_x, start_pos_y, end_pos_x, end_pos_y)) {
				if (end > 0) {
					end--;
					end_pos_x = x_path[end];
					end_pos_y = y_path[end];
				}
				else {
					break;
				}
			}
			x_path_smooth[pointnumber_smooth] = x_path[end];
			//			cout << "- -  " << x_path_smooth[pointnumber_smooth];
			y_path_smooth[pointnumber_smooth] = y_path[end];
			//			cout << "- -" << y_path_smooth[pointnumber_smooth] << endl;
			pointnumber_smooth++;
			start = end;
		}
		printf("pointnumber_smooth is %d\n", pointnumber_smooth);

		//		std::pair<int, int> temp;
		//		for (int i = 0; i < pointnumber_smooth-1; i++)
		//		{
		//			temp.first = x_path_smooth[i];
		//			temp.second = y_path_smooth[i];
		////			path_new.push_back(temp);

		//		}
		//cout << pointnumber_smooth << endl;
		//x_path_smooth[pointnumber_smooth] = x_path[pointnumber - 1];
		//y_path_smooth[pointnumber_smooth] = y_path[pointnumber - 1];
		//pointnumber_smooth++;
	////		cout << x_path_smooth[0] << " " << y_path_smooth[0] << endl;
		printf("Path planning success! \n");
		return true;
	}

} //end namespace my_planner

bool  RrtPath::checkIfOnObstacles(int node_x, int node_y) {

	double  get_nearest = get_nearest_distance(node_x, node_y);

	if (get_nearest < lethal_distance) {
		return true;
	}
	else {
		return false;
	}
}

bool  RrtPath::check_if_on_obstacle(int rrtnode_x, int rrtnode_y, int node_x, int node_y) {
	point test;
	float p = 0.00;
	test.parent = 400;

	double dist = sqrt(pow(rrtnode_x - node_x, 2) + pow(rrtnode_y - node_y, 2));
	double theta = atan2(node_y - rrtnode_y, node_x - rrtnode_x);
	int k = 0;
	while (p <= dist) {
		test.pos_x = int(rrtnode_x + p * cos(theta));
		test.pos_y = int(rrtnode_y + p * sin(theta));
		k++;
		if (checkIfOnObstacles(test.pos_x, test.pos_y))
		{
			return true;
			break;
		}
		p = p + extend_check;
	}
	return false;
}

/*
int**readText(string file){
	ifstream infile;
	infile.open(file.data());   //将文件流对象与文件连接起来
	assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

	while (!infile.eof())
	{
		for (int i = 0; i < 450; i++) {
			for (int j = 0; j < 600; j++) {
				infile >> costs[j][i];
			}
		}

	 }
	infile.close();             //关闭文件输入流
	return costs;
}
*/
//int main() {
//	//readText("map.txt");
//	RrtPath rrtpath;
//	if (rrtpath.getPath(costs, 50, 50, 550, 550, path)) cout << "1";
//	else cout << "0";
//
//	vector<pair<int, int> > ::iterator iter;
//	for (iter = path.begin(); iter != path.end(); iter++) {
//		cout << (*iter).first << (*iter).second << endl;
//	}
//}