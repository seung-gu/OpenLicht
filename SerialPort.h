#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define END_BIT 255

class SerialPort  
{
public:
	static void setUART();
	static void transmitting(unsigned char* data, int size);
	static void receiving();
	static void closeUART();
private:
	static int uart0_filestream;
};
#endif
