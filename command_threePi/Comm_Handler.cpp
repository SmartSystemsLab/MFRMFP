#include <iostream>
#include <SerialPort.h>
#include "SSL_comm.h"
using namespace std;

class Comm_handler //class name
{
	private:
		SerialPort* mySerial; //access specifier: private member
	public: 
		Comm_handler(); //constructor, connect Serial port
		~Comm_handler(); //destructor, disconnect Serial port
		void send_command(unsigned int des, command data); 
	  bool request_sensor(unsigned int des, sensor *data); 
};

Comm_handler::Comm_handler()
{
	mySerial = new SerialPort("/dev/ttyUSB0");
	mySerial->Open();
	mySerial->SetBaudRate(SerialPort::BAUD_9600);
}

Comm_handler::~Comm_handler()
{
	mySerial->Close();
	delete mySerial;
}

void Comm_handler::send_command(unsigned int des, command data)
{
    unsigned char payload[128];
	  unsigned char frame[128];
	  int len_c = pack_command(payload, data);
    int frame_len = build_frame(frame, payload, des, len_c);
  
    for (int i = 0; i < frame_len; i++)
    {
      mySerial->WriteByte(frame[i]);
    }
}

bool Comm_handler::request_sensor(unsigned int des, sensor *data)
{
	unsigned char frame[128];
	unsigned char payload[128];
	char strength;
    payload[0] = REQ; //read first byte of payload, request sensor data
    int frame_len = build_frame(frame, payload, des, 1);
	
	
	for (int i = 0; i < frame_len; i++)
	{
	  mySerial->WriteByte(frame[i]);
	}
  
	while(1)
	{
	    while(!mySerial->IsDataAvailable()) {};
    
	    frame[0] = mySerial->ReadByte();
    
	    if (frame[0] == 0x7E)
	    {
	      break;
	    }
	 }
  
	 frame[1] = mySerial->ReadByte(100);
	 frame[2] = mySerial->ReadByte(100);
	 unsigned int len = ((unsigned int)frame[1]<<8) + frame[2];
	 
	 for(int i = 3; i < len+4; i++)
	 {
	 	frame[i] = mySerial->ReadByte(100);
	 }
	  
	 int extract_payload = extract_data(frame, len+4, payload, &des, &strength);
	 if(extract_payload == -1)
	 {
	 	return false;
	 }
	 else
	 {
	 	unpack_sensor(payload, data);
	 	return true;
 	 }
}

int main()
{
	Comm_handler Commhand; //object name
	command data1;
	sensor data2;
	Commhand.send_command(2, data1); 
	Commhand.request_sensor(2, &data2);
    cout<< data2.x << endl;
    cout<< data2.y << endl;
    cout<< data2.z << endl;
    cout<< data2.l_eco << endl;
    cout<< data2.r_eco << endl;
}
	
