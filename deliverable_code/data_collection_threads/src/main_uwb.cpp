#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <time.h>
#include <sstream>
#include <typeinfo>
#include <math.h>
#include <fstream>
#include <unistd.h>
//#include "ems_data_structures.h"

using namespace std;
//~ using namespace cv;

void port_open(void);
void port_close(void);
void port_init(void);
void port_read(string data);
int configure_tag();

typedef double	Double64;

int fdUART; /*File Descriptor*/

double uwb_time_tag, time_tag;
int counter = 1;
char uwb_start[5] = "DIST";
unsigned char uwb_init_wr[] = {0x0D, 0x0D};
unsigned char uwb_next_wr[] = {'l', 'e', 'c', 0x0D};

char csv_file_path[100];
const int result = remove("../Data/uwb_data.csv");
ofstream uwb_data_file("../Data/uwb_data.csv", ios::app);

auto time_start = std::chrono::system_clock::now();
auto time_now = std::chrono::system_clock::now();

int main_uwb(bool & e,string &data) {
	cout << "\nWelcome to UWB Serial Reading Project!";
   
	port_open();
	port_init();
			
	time_t temp = chrono::system_clock::to_time_t(time_start);
	cout << "\nStart Time: " << ctime(&temp);
	
	int bytes_written = configure_tag(); // Configure for CSV output
	
	if (bytes_written)
	{	
		cout << "IN"<<endl;
		while(!e)
		usleep(10);
		while (e){
		try{
			port_read(data);
			cout<<"\n*Count : "<<counter<<endl;
			++counter;
			}
			catch(...){
			printf("catch .....................................................\n");
			}
		}
	}
	
	// while (1){
	// 	port_read();
	// 	cout<<"\n*Count : "<<counter<<endl;
	// 	++counter;
	// }
	
	port_close();
	uwb_data_file.close();
   
	return 0;
}

int configure_tag()
{
	int bytes_written_1 = write(fdUART, uwb_init_wr, sizeof(uwb_init_wr));
	sleep(3);
	int bytes_written_2 = write(fdUART, uwb_next_wr, sizeof(uwb_next_wr));
	sleep(1);

	return bytes_written_2;
}

void port_read(string data){
	
	int b_size = 1000;
	char read_buffer[b_size];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	tcflush(fdUART, TCIFLUSH);   /* Discards old data in the rx buffer            */

	bytes_read = read(fdUART, &read_buffer, sizeof(read_buffer)); /* Read the data                   */
	
	time_now     = std::chrono::system_clock::now();
	uwb_time_tag = 	chrono::duration<double>(time_now-time_start).count();
	//~ cout << "\nTime from Start: " << chrono::duration<double>(time_now-time_start).count() <<endl;
	
	//~ read_buffer[bytes_read] = 0; //terminate the string

	
	cout << "\nTime Now: " << uwb_time_tag <<endl;
	char *uwb_chararr = strstr(read_buffer, uwb_start);
	std::string uwb_substr(uwb_chararr);
	std::size_t pos = uwb_substr.find(0x0D);
	 data = uwb_substr.substr(0, pos);
	
	// cout << data ;
	uwb_data_file << uwb_time_tag << "," << data << endl;

}



void port_close(void){
	if (close(fdUART) <0) /* Close the serial port */	
		cout <<"\nClosing port Error!"; 
	
	}

void port_open(void){
	
/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
/* O_RDWR   - Read/Write access to serial port       */
/* O_NOCTTY - No terminal will control the process   */
/* Open in blocking mode,read will wait              */					                                        
		
	const char *portName = "/dev/ttyACM0";
								
	if((fdUART = open(portName, O_RDWR | O_NOCTTY))== 1)						/* Error Checking */
		cout << "\nSeroal port on " <<portName<<" cannot be opened!";	

	}
	
void port_init(void){
	/*---------- Setting the Attributes of the serial port using termios structure --------- */
	
	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fdUART, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 115200                     */
	cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 115200                       */

	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
	
	//SerialPortSettings.c_iflag = IGNPAR;
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);          /* Disable XON/XOFF flow control both i/p and o/p */
	
	
	 /* disable canonical input, disable echo,
	 disable visually erase chars,
	 disable terminal-generated signals */
	 SerialPortSettings.c_lflag &= ~ICANON;
	 SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);
	 /* disable output processing */
	 SerialPortSettings.c_oflag &= ~OPOST;
	
	SerialPortSettings.c_cc[VTIME]    = 5;   /* inter-character timer unused */
    SerialPortSettings.c_cc[VMIN]     = 750;   /* 184: blocking read until 5 chars received */
	tcflush(fdUART, TCIFLUSH);   /* Discards old data in the rx buffer            */

	tcsetattr(fdUART,TCSANOW,&SerialPortSettings);
	if((tcsetattr(fdUART,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	    cout << "\nSerial port initialization Error!";	
	}
