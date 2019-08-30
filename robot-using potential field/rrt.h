
#pragma once
#ifndef MY_PLANNER_RTT_H
#define MY_PLANNER_RTT_H

#include<vector>
// #include<global_planner/traceback.h>
// #include <global_planner/planner_core.h>
#include <cmath>
#include<algorithm>
#include <utility>  

#define POT_HIGH 500;
#define xs_ 600; //待加
#define ys_ 450; //待加
using namespace std;
extern  vector<std::pair<double, double> > path;
extern  vector<pair<double, double> >  path_new;
extern  int map_cost[450][600];
extern int map_blue_obstacles_x[8];
extern  int map_blue_obstacles_y[8];
extern  int map_yellow_obstacles_x[8];
extern  int map_yellow_obstacles_y[8];
const double extend_check = 0.8;
//namespace my_planner {



struct _point {
	int parent;
	int pos_x;
	int pos_y;
};

typedef struct _point point;

class RrtPath //: public Traceback
{
public:
	//    RrtPath(PotentialCalculator *p_calc, bool use_auto_path) : Traceback(p_calc,use_auto_path) {}
	int(*costs)[600];
	double step;
	double lethal_distance;
	int * map_blue_obstacles_x;
	int * map_blue_obstacles_y;
	int * map_yellow_obstacles_x;
	int * map_yellow_obstacles_y;
	int test_car_number;
	int number_map_blue_obstacles;
	int number_map_yellow_obstacles;

	double get_nearest_distance(int test_x, int test_y);
	//double* get_nearest_point(int test_x, int test_y);
	bool  getPath(int start_x, int start_y, int end_x, int end_y);
	bool checkIfOnObstacles(int node_x, int node_y);
	bool check_if_on_obstacle(int rrtnode_x, int rrtnode_y, int node_x, int node_y);
	inline double distance(point x, point y) {
		return sqrt(pow(fabs(x.pos_x - y.pos_x), 2) + pow(fabs(x.pos_y - y.pos_y), 2));
	}

	/*返回的是向量元素序号*/
	int get_nearest(std::vector<point> &seeds, point seed) {
		int result = 0;
		double minimun = POT_HIGH;
		for (int i = 0; i < (int)seeds.size(); i++) {
			double dis = distance(seeds[i], seed);
			if (dis < minimun) {
				result = i;
				minimun = dis;
			}
		}
		return result;
	}
	RrtPath();
	~RrtPath();

	int* get_x_path_smooth() {
		return x_path_smooth;
	}
	int* get_y_path_smooth() {
		return y_path_smooth;
	}
	int * get_x_path() {
		return x_path;
}

	int * get_y_path() {
		return y_path;
	}

	int get_pointnumber_smooth() {
		return pointnumber_smooth;
	}

	int get_pointnumber() {
		return pointnumber;
	}
	//		vector<pair<int, int> >  path_new;
private:
	int ns_;
	int  x_path[600];
	int  y_path[450];
	int x_path_smooth[600];
	int y_path_smooth[450];
	int  pointnumber;
	int pointnumber_smooth;


};

//} //end namespace my_planner

#endif //MY_PLANNER_RTT_H
