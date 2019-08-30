#include "udp_commu_recv.h"
udp_commu_recv::udp_commu_recv() {
	init();
}
void udp_commu_recv::init() {

	rfd = fd_set();
	timeout.tv_sec = 0;               //等下select用到这个
	timeout.tv_usec = 0;              //timeout设置为0，可以理解为非阻塞
	int Error;
	WORD VersionRequested;
	WSADATA WsaData;
	VersionRequested = MAKEWORD(2, 2);
	Error = WSAStartup(VersionRequested, &WsaData); //启动WinSock2
	if (Error != 0)
	{
		cout << "error" << endl;
	}
	else
	{
		if (LOBYTE(WsaData.wVersion) != 2 || HIBYTE(WsaData.wHighVersion) != 2)
		{
			WSACleanup();
			cout << "error" << endl;
		}
	}
	int recvbuf = 1;
	s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	setsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&recvbuf, sizeof(int));

	sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(23333);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	::bind(s, (sockaddr*)&addr, sizeof(addr));
}

udp_commu_recv::~udp_commu_recv() {
	
}

void udp_commu_recv::mainloop() {
		Recv();
}


void udp_commu_recv::Recv(int flag)
{


	char buff[2000] = {};

	if (flag == 1)
	{
		int rec = recv(s, buff, 2000, 0);
		//cout << rec << endl;
		while (recv<=0)
		{

			rec = recv(s, buff, 2000, 0);
			//cout << rec << endl;
		}
	}
	else
	{
		int SelectRcv;
		FD_ZERO(&rfd);           // 总是这样先清空一个描述符集
		FD_SET(s, &rfd); // 把sock放入要测试的描述符集
		SelectRcv = select(s + 1, &rfd, 0, 0, &timeout); //检查该套接字是否可读
		if (SelectRcv < 0)
		{
			return;
		}
		else
		{
			while (SelectRcv) //实时更新
			{
				int rec;
				rec = recv(s, buff, 2000, 0);
				FD_ZERO(&rfd);           //总是这样先清空一个描述符集
				FD_SET(s, &rfd); //把sock放入要测试的描述符集
				SelectRcv = select(s + 1, &rfd, 0, 0, &timeout); //检查该套接字是否可读
				//cout << "rec" << rec << endl;
				//cout << SelectRcv << endl;
			}
		}
	}

	recv_num = vision_recv.ParseFromArray(buff, sizeof(buff));
	if (recv_num < 0)
	{
		perror("recvfrom error:");
		exit(1);
	}
	//cout << "recv success" << endl;
	ball = vision_recv.balls();
	//cout << vision_recv.robots_yellow_size() << endl;
	robots_y.clear();
	robot_yellow.clear();
	for (int i = 0; i < vision_recv.robots_yellow_size(); i++) {
		robots_y.push_back(vision_recv.robots_yellow(i));

		robot_yellow.push_back(robot_info{ robots_y[i].valid(),robots_y[i].robot_id(),robots_y[i].x(),robots_y[i].y(),robots_y[i].orientation(),robots_y[i].vel_x(),robots_y[i].vel_y(),robots_y[i].rotate_vel(),robots_y[i].accelerate_x(),robots_y[i].accelerate_y(),robots_y[i].raw_x(),robots_y[i].raw_y(),robots_y[i].raw_orientation(),robots_y[i].raw_vel_x(),robots_y[i].raw_vel_y(),robots_y[i].raw_rotate_vel() });
		//	cout << "recive:" << robots_y[i].robot_id() << " " << robots_y[i].x() << " " << robots_y[i].y() << endl;

	}
	robots_b.clear();
	robot_blue.clear();
	for (int i = 0; i < vision_recv.robots_blue_size(); i++) {
		robots_b.push_back(vision_recv.robots_blue(i));
		robot_blue.push_back(robot_info{ robots_b[i].valid(),robots_b[i].robot_id(),robots_b[i].x(),robots_b[i].y(),robots_b[i].orientation(),robots_b[i].vel_x(),robots_b[i].vel_y(),robots_b[i].rotate_vel(),robots_b[i].accelerate_x(),robots_b[i].accelerate_y(),robots_b[i].raw_x(),robots_b[i].raw_y(),robots_b[i].raw_orientation(),robots_b[i].raw_vel_x(),robots_b[i].raw_vel_y(),robots_b[i].raw_rotate_vel() });
		if (robots_b[i].robot_id() == my_id) {
			my_robot = robot_info{ robots_b[i].valid(),robots_b[i].robot_id(),robots_b[i].x(),robots_b[i].y(),robots_b[i].orientation(),robots_b[i].vel_x(),robots_b[i].vel_y(),robots_b[i].rotate_vel(),robots_b[i].accelerate_x(),robots_b[i].accelerate_y(),robots_b[i].raw_x(),robots_b[i].raw_y(),robots_b[i].raw_orientation(),robots_b[i].raw_vel_x(),robots_b[i].raw_vel_y(),robots_b[i].raw_rotate_vel() };
			my_robot.x = my_robot.x / 10 + 300;
			my_robot.y = my_robot.y / 10 + 225;
			my_robot.v_x /= 10;
			my_robot.v_y /= 10;
		}
	}

	//cout << robot_yellow[0].v_x;
	robo_ball = { ball.vel_x(),ball.vel_y(),ball.area(),ball.x(),ball.y(),ball.height(),ball.ball_state(),ball.last_touch(),ball.valid(),ball.raw_x(),ball.raw_y(),ball.chip_predict_x(),ball.chip_predict_y() };
	//::closesocket(s);
	//::WSACleanup();
	//system("pause");
}
