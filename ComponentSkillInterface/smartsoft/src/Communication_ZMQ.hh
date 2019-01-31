//--------------------------------------------------------------------------
//
//  Copyright (C)	2018 Matthias Lutz
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//--------------------------------------------------------------------------

#ifndef _COMMUNICATION_ZMQ_HH
#define _COMMUNICATION_ZMQ_HH

#include <string>
#include <zmq.hpp>
#include "ParameterStateStruct.hh"
#include "Communication.hh"

class Communication_ZMQ : public Communication
{
private:

	SmartACE::SmartRecursiveMutex writeComLock;

	bool connected;
	std::string ip;
	unsigned int port;

	bool verbose;

	int recv(std::string & msg);

public:

	int recv_lines(std::deque<std::string> & deque);
	int send(const std::string& message);

	Communication_ZMQ();
	bool setConfig(std::string ip, unsigned int port, bool use_socket_timeout, unsigned int timeout_value_sec, bool verbose = false);
	int connect();
	int disconnect();

	zmq::context_t context;
	zmq::socket_t request_socket;
	zmq::socket_t reply_socket;

	virtual ~Communication_ZMQ();

};

#endif
