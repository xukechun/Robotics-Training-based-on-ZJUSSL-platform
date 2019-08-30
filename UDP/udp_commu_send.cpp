#include "udp_commu_send.h"
udp_commu_send::udp_commu_send() {
	init();
}
void udp_commu_send::init() {
	/*cv pic*/
	
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		cout << "Initialization failed." << endl;
	}
	SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (SendSocket == -1) {
		cout << "Socket failed." << endl;
		
	}
	addr_client.sin_family = AF_INET;
	addr_client.sin_port = htons(50001);
	//addr_client.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addr_client.sin_addr.s_addr = inet_addr("127.0.0.1");

}

udp_commu_send::~udp_commu_send() {

}



void udp_commu_send::send(float id, float x, float y, float r, bool kick, float power, float spin) {

	//init();
	//char buffer[10240];

	long Port = 50001;
	//char SendBuf[1024 * 10];
	//int BufLen = 1024 * 10;

	//WSAStartup(MAKEWORD(2, 2), &wsaData);
	//SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//addr_client.sin_family = AF_INET;
	//addr_client.sin_port = htons(Port);
	//addr_client.sin_addr.s_addr = inet_addr("127.0.0.1");

	Robots_Command rcs;
	rc = rcs.add_command();
	rc->set_robot_id(id);
	rc->set_velocity_x(x);
	rc->set_velocity_y(-y);
	rc->set_velocity_r(r);
	rc->set_kick(kick);
	rc->set_power(power);
	rc->set_dribbler_spin(spin);
	int sock_fd;
	int size = rcs.ByteSize();
	char *buffer = new char[size];
	rcs.SerializeToArray(buffer, size);

	int a = sendto(SendSocket, buffer, size, 0, (SOCKADDR*)& addr_client, sizeof(addr_client));
	//cout << "send:             " << a <<endl;
	Sleep(30);
	delete []buffer;
	//::closesocket(SendSocket);
	//::WSACleanup();
}

void udp_commu_send::Send_debug(std::vector <std::pair<int, int>> path)
{
	char buffer[1024];
	int buffer_size = 1024;
	Debug_Msgs lines_set;
	std::vector <std::pair<int, int>>::iterator itev = path.begin();
	for (; itev != path.end() - 1; ++itev) {
		int sx = (*itev).first - 300;
		int sy = 225 - (*itev).second;
		int ex = (*(itev + 1)).first - 300;
		int ey = 225 - (*(itev + 1)).second;
		Debug_Msg* line_set;
		Debug_Line *line = new Debug_Line();
		Point *start = new Point();
		Point *end = new Point();

		start->set_x(sx);
		start->set_y(sy);
		end->set_x(ex);
		end->set_y(ey);
		line->set_allocated_start(start);
		line->set_allocated_end(end);
		line->set_forward(true);
		line->set_back(false);
		line_set = lines_set.add_msgs();
		line_set->set_allocated_line(line);
		line_set->set_type(Debug_Msg_Debug_Type_LINE);
		line_set->set_color(Debug_Msg_Color_YELLOW);
	}

	lines_set.SerializeToArray(buffer, buffer_size);
	WSADATA wsaData;
	sockaddr_in RecvAddr;
	int Port = 20001;

	char SendBuf[1024 * 10];
	int BufLen = 1024 * 10;

	WSAStartup(MAKEWORD(2, 2), &wsaData);

	SOCKET SendSocket;
	SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_port = htons(Port);
	RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	sendto(SendSocket, buffer, BufLen, 0, (SOCKADDR*)& RecvAddr, sizeof(RecvAddr));

}
