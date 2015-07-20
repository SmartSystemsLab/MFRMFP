/*
 * command_threePi
 *
 * commands wheel velocities to the threePi
 */

#include "SSL_comm.hpp"
#include <iostream>
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
	Comm_handler comms;
	
	cmd.vl = atof(argv[1]);
	cmd.vr = atof(argv[2]);
	dest = atoi(argv[3]);
	
	cout << "sending: v_l=" << cmd.vl <<" v_r=" << cmd.vl << " to " << dest << endl;
	comms.send_command(dest, cmd);
	
	return 0;
}

