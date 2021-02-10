/*
 * Socket.h
 *
 *  Created on: Jan 18, 2011
 *      Author: wopfner
 */

#ifndef SOCKET_H_
#define SOCKET_H_

#include <string>
#include <stdint.h>

#ifdef WITH_MRPT_2_0_VERSION
#include <mrpt/comms/CClientTCPSocket.h>
#else
#include <mrpt/utils/CClientTCPSocket.h>
#endif
class Socket {

public:
	class SocketException: std::exception {
	private:
		std::string message;

	public:
		SocketException(const std::string& m) throw() {
			message = m;
		}

		virtual ~SocketException() throw() {
		}

		virtual const char* what() const throw() {
			return message.c_str();
		}

	};

private:
	bool connected;
	int sockDesc;

#ifdef WITH_MRPT_2_0_VERSION
	mrpt::comms::CClientTCPSocket socket;
#else
	mrpt::utils::CClientTCPSocket socket;
#endif

public:
	Socket();
	virtual ~Socket();

	bool connect(const std::string& host, int port);
	bool close();

	void write(const char* buf, size_t size);
	size_t read(char* buf, size_t size);

	bool readLine(std::string& out);

	bool isConnected() {
		return socket.isConnected();
	}
};

#endif /* SOCKET_H_ */
