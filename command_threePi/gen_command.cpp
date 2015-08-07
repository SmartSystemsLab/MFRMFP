/*
 * gen_command
 *
 * commands wheel velocities to the threePi
 */

#include "SSL_comm.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
	if (argc != 4)
	{
		cout << "usage: " << argv[0] << " <left vel> <right vel> <destination address>" << endl;
		return 0;
	}
	
	command cmd;
	unsigned int dest;
	unsigned char payload[128];
	unsigned char pkt[128];
	int pay_len;
	int pkt_len;
	
	cmd.vl = atof(argv[1]);
	cmd.vr = atof(argv[2]);
	dest = atoi(argv[3]);
	
	pay_len = pack_command(payload, cmd);
	pkt_len = build_frame(pkt, payload, dest, pay_len);
	
	cout << "Packet: ";
	for (int i = 0; i < pkt_len; i++)
	{
		printf("% .2x ", pkt[i]);
	}
	
	cout << endl;
	
	return 0;
}

