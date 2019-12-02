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

#ifndef _COMMUNICATION_SOCK_HH
#define _COMMUNICATION_SOCK_HH

#include "ace/INET_Addr.h"
#include "ace/SOCK_Connector.h"
#include "ace/SOCK_Acceptor.h"
#include "ace/SOCK_Stream.h"
#include "ace/SOCK_Dgram.h"
#include "ace/Event_Handler.h"
#include <string>
#include "ParameterStateStruct.hh"
#include "Communication.hh"

class Communication_sock : public Communication ,public ACE_Event_Handler, public SmartACE::TimerHandler
{
private:

	SmartACE::SmartRecursiveMutex writeComLock;

	ACE_SOCK_Connector connector;
	ACE_SOCK_Stream peer;
	ACE_INET_Addr server_addr;
    ACE_INET_Addr peer_addr;
	bool connected;

	int connectionTimerId;
	bool connection_timeout;

	std::string ip;
	unsigned int port;
	bool use_socket_timeout;
	unsigned int timeout_value_sec;

	bool verbose;

	int handle_signal (int signum, siginfo_t *, ucontext_t *);
	virtual void timerExpired(const Smart::TimePoint &abs_time, const void * arg) override;
	int recv(std::string & msg);


	/// Receiving line stuff
	std::string messageRest;
	std::string splitIntoLines(std::deque <std::string> &result, std::stringstream &ss);
	void removeMessagesWithNonePrintChars(std::deque <std::string> &result);


public:

	int recv_lines(std::deque<std::string> & deque);
	int send(const std::string& message);

	Communication_sock();
	bool setConfig(std::string ip, unsigned int port, bool use_socket_timeout, unsigned int timeout_value_sec, bool verbose = false);
	int connect();
	int disconnect();


	~Communication_sock();

};

#endif
