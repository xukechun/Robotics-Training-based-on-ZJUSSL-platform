#include "vision_detection.pb.h"
#include "zss_cmd.pb.h"
using namespace std;
#include <iostream>
#include <winsock2.h>
#pragma comment(lib, "WS2_32")
#include <Windows.h>
// #include "zss_debug.pb.h"
using namespace std;
#define DSET_IP_ADDRESS  "127.0.0.1"   
#define SERV_PORT   50001   
#define DEST_PORT  50001 
class udp_commu_recv
{
public:
	udp_commu_recv();
	~udp_commu_recv();
	int sock_fd; 

	Vision_DetectionFrame vision_recv;
	vector<Vision_DetectionRobot> robots_y;
	vector<Vision_DetectionRobot> robots_b;
	Vision_DetectionBall ball;
	Robots_Command robotc;
	Robot_Command* robot;
	struct robot_info{
		bool valid;
		unsigned int id;
		float x;
		float y;
		float orientation;
		float v_x;
		float v_y;
		float rotate_vel;
		float accelerate_x;
		float accelerate_y;
		float raw_x;
		float raw_y;
		float raw_orientation;
		float raw_vel_x;
		float raw_vel_y;
		float raw_rotate_vel;
		
	};
	struct ball_info{
		float v_x;
		float v_y;
		unsigned int area;
		float x;
		float y;
		float height;
		unsigned int ball_state;
		unsigned int last_touch;
		bool valid;
		float raw_x;
		float raw_y;
		float chip_predict_x;
		float chip_predict_y;
	};

	vector<robot_info> robot_blue;
	vector<robot_info> robot_yellow;
	ball_info robo_ball;
   // Mat
	robot_info my_robot;
	int my_id;
	void init();
	void mainloop();
	void Recv(int flag = 0);


	 int recv_num;  
	 WSADATA data;
	 SOCKET s;
  int send_num;  
  char send_buf[1024] ;  
  char recv_buf[65536];  
  struct sockaddr_in addr_client;  
  struct sockaddr_in addr_serv;  
  int len;
  fd_set rfd;
  struct timeval timeout;

};