#include "Bluetooth.h"

int Bluetooth::s, status;

void Bluetooth::setBluetooth()
{
	struct sockaddr_rc addr = {0};
	
	char dest[18] = "98:D3:32:21:2E:EA";	//bluetooth module
	
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);	//bluetooth module paring
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba(dest, &addr.rc_bdaddr);
	
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
}
void Bluetooth::transmitting(unsigned char* _data, int size)
{
	unsigned char tx_data[size+1];
	for(int i=0; i<size; i++)
		tx_data[i] = _data[i];
	tx_data[size] = END_BIT;
	
	status = write(s, tx_data, size+1);

	//if(status<0) perror("ERROR");
}
void Bluetooth::closeBluetooth()
{
	close(s);
}
