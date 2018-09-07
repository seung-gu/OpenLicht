#ifndef BLUETOOTH_SERVER_H
#define BLUETOOTH_SERVER_H

#include <stdio.h>
#include <stdlib.h>

#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>

class Bluetooth_server
{
public:
	static int _str2uuid(const char *uuid_str, uuid_t *uuid);
	static sdp_session_t* register_service(uint8_t rfcomm_channel);
	static int init_server();
	static char* read_server(int client);
	static void write_server(int client, char *message);
private:
	static bdaddr_t bdaddr_any;
	static bdaddr_t bdaddr_local;
	static char input[1024];
};

#endif