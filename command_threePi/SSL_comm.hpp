/*
 * SSL_comm.hpp
 *
 * header file that handels communication between parent computer and three pi
 * robots.
 */

// Includes
#include <SerialPort.h>
#include <stdio.h>

using namespace std;

// Macros
#define CMD 0x01
#define SND 0x02
#define REQ 0x03

// Structs
struct sensor
{
	float x;
	float y;
	float z;
	int l_eco;
	int r_eco;
};
	
struct command
{
	float vl;
	float vr;
};

// Classes
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

// function prototypes
int pack_sensor(unsigned char* payload, sensor data);
/*
 * pack_sensor
 * Description: Packs data into an array to be sent over serial.
 * Parameter list:
 *  payload - array that the data will be packed into
 *  data    - struct containing data to be packed
 * Returns: length of the payload array
 */
void unpack_sensor(unsigned char* payload, sensor* data);
/*
 * unpack_sensor 
 * Description: Unpacks received data from an array 
 * Parameter list:
 * payload - array that the data will be unpacked from
 * data - struct containing data to be unpacked
*/
int pack_command(unsigned char* payload, command data);
/*
 * pack_command
 * Description: Packs data into an array to be sent over serial.
 * Parameter list:
 *  payload - array that the data will be packed into
 *  data    - struct containing data to be packed
 * Returns: length of the payload array
 */
void unpack_command(unsigned char* payload, command* data);
/*
 * unpack_command 
 * Description: Unpacks received data from an array 
 * Parameter list:
 * payload - array that the data will be unpacked from
 * data - struct containing data to be unpacked
 */
int build_frame(unsigned char* frame,unsigned char* payload, unsigned int des, int payload_size);
/*
 * build_frame
 * Description: build a frame for the packet to be sent over serial
 * Parameter list:
 *  frame       - array that stores the elements in a packet
 *  payload     - array that stores the essential information of the packet
 *   des        - destination address of the packet
 * payload_size - length of the payload
 */
int extract_data(unsigned char* frame, unsigned int frame_size, unsigned char* payload, unsigned int* src, char* strength);
/*
 * extract_data
 * Description: Extracts data from the packet upon receiving 
 * Parameter list:
 *    frame    - array that contains elements in a packet
 *  frame_size - length of the frame 
 *    payload  - array that stores the essential information of the packet
 *     src     - source address
 *   Strength  - signal strength
 */

// template definitions
template <class T>
void obj2bytes(T* obj, unsigned char* str)
{
	char* ptr = (char*)obj;
	int size = sizeof(T);
	
	for(int i = 0; i < size; i++)
	{
		str[i] = ptr[i];
	}
}

template <class T>
void bytes2obj(T* obj, unsigned char* str)
{
	char* ptr = (char*)obj;
	
	for(int i = 0; i < sizeof(T); i++)
	{
		ptr[i] = str[i];
	}
}

// function definitions
int pack_sensor(unsigned char* payload, sensor data)
{
	int len_s = sizeof(sensor) + 1;
	payload[0] = SND;
	obj2bytes<sensor>(&data, &payload[1]);
	return len_s;
}

void unpack_sensor(unsigned char* payload, sensor* data)
{
	if(payload[0] == SND)
	{
		bytes2obj<sensor>(data, &payload[1]);
	}
}

int pack_command(unsigned char* payload, command data)
{
	unsigned int len_c = sizeof(command)+1;
	payload[0] = CMD;
	obj2bytes<command>(&data, &payload[1]);
	return len_c;
}

void unpack_command(unsigned char* payload, command* data)
{
	if(payload[0] == CMD)
	{
		bytes2obj<command>(data, &payload[1]);
	}
}

int build_frame(unsigned char* frame,unsigned char* payload, unsigned int des, int payload_size)
{
	unsigned int len = 5 + payload_size; //8 bytes plus payload_size
	unsigned char chksum = 0;
	
	frame[0] = 0x7E; //frame delimiter, byte 1
	//length- store MSB, then LSB, byte 2&3
	frame[1] = (unsigned char)(len>>8) & 0x00FF; //shift len left 8 bits, getting the most significant byte
	frame[2] = (unsigned char)(len) & 0x00FF; //least significant byte
	frame[3] = 0x01; //API Identifier value , byte 4
	frame[4] = 0; //Frame ID, byte 5, assigned '0' to disable response frame
	//Destination Address, byte 6&7, MSB first, LSB last, Broadcast = 0xFFFF
	frame[5] = (unsigned int)(des>>8) & 0x00FF; 
	frame[6] = (unsigned int)(des) & 0x00FF;
	frame[7] = 0; //Options, byte 8

	for(int i = 0; i < payload_size; i++)
	{
		frame[i + 8] = payload[i];
	}

	for(int i = 3; i < len+3; i++)
	{
		chksum += frame[i];
	}
	frame[len+3] = 0xFF - chksum;
	return len+4;
}

int extract_data(unsigned char* frame, unsigned int frame_size, unsigned char* payload, unsigned int* src, char* strength)
{
	unsigned char chksum_1 = 0;
	for(int i = 3; i < frame_size - 1; i++)
	{
		chksum_1 += frame[i];
	}
	chksum_1 = 0xFF - chksum_1;
	if (chksum_1 != frame[frame_size - 1])
	{
		return -1;
	}
	*strength = frame[6];
	*src = ((unsigned int)frame[4]<<8) + frame[5]; 
	int payload_size = frame_size - 9;
	for(int i = 0; i < payload_size; i++)
	{
		payload[i] = frame[i + 8];
	}
	
	return payload_size;
}

Comm_handler::Comm_handler()
{
	mySerial = new SerialPort("/dev/ttyUSB0");
	mySerial->Open();
	mySerial->SetBaudRate(SerialPort::BAUD_57600);
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
    
    printf("Packet:\n");
    for (int i = 0; i < frame_len; i++)
    {
      mySerial->WriteByte(frame[i]);
      printf("% .2x ", frame[i]);
    }
    
    printf("\n");
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

