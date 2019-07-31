#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>

#define END_BIT 255

class Bluetooth
{
public:
	static void setBluetooth();
	static void transmitting(unsigned char* data, int size);
	static void closeBluetooth();
private:
	static int s, start;
};
#endif
