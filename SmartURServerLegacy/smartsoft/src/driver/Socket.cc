/*
 * Socket.cpp
 *
 *  Created on: Jan 18, 2011
 *      Author: wopfner
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <iostream>

#include "Socket.hh"

Socket::Socket()
{
	connected = false;
}

Socket::~Socket()
{
	close();
}

bool Socket::connect(const std::string& host, int port)
{
	try
	{
		socket.connect(host, port, 10000);
	} catch (...)
	{
		std::cout << " ERROR\n";
		return false;
	}

	return socket.isConnected();
}

bool Socket::close()
{
	socket.close();
	return !socket.isConnected();
}

void Socket::write(const char* buf, size_t size)
{

	//DEBUG:
//	printf("%s", buf);
	///

	if (isConnected())
	{
		socket.writeAsync(buf, size);
	} else
	{
		throw SocketException("Socket not connected!");
	}
}

size_t Socket::read(char* buf, size_t size)
{
	if (isConnected())
	{
		return socket.readAsync(buf, size);
	} else
	{
		throw SocketException("Socket not connected!");
	}
}

bool Socket::readLine(std::string& out){
	return socket.getline(out);
}
