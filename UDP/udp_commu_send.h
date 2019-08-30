#include "vision_detection.pb.h"
#include "zss_cmd.pb.h"
using namespace std;
#include <iostream>
#include <winsock2.h>
#include "zss_debug.pb.h"
// #include "zss_debug.pb.h"
using namespace std;
#define DSET_IP_ADDRESS  "127.0.0.1"   
#define SERV_PORT   50001   
#define DEST_PORT  50001 

class udp_commu_send
{
public:
	udp_commu_send();
	~udp_commu_send();
	
	Vision_DetectionFrame vision_recv;
	vector<Vision_DetectionRobot> robots_y;
	vector<Vision_DetectionRobot> robots_b;
	Vision_DetectionBall ball;

	Robot_Command* rc;
	struct robot_info{
		bool valid;
		int id;
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
		int area;
		float x;
		float y;
		float height;
		float ball_state;
		float last_touch;
		float valid;
		float raw_x;
		float raw_y;
		float chip_predict_x;
		float chip_predict_y;
	};
	vector<robot_info> robot_blue;
	vector<robot_info> robot_yellow;
	ball_info robo_ball;
   // Mat
	vector<Point> blue_robot;
	vector<Point> yellow_robot;
	void init();
	void send( float id, float x, float y, float r, bool kick, float power, float spin);
	void Send_debug(std::vector <std::pair<int, int>> path);

 	//Mat map;
	 int recv_num;  
  int send_num;  

  struct sockaddr_in addr_client;  
  int len;  
  WSADATA wsaData;
  SOCKET SendSocket;
};