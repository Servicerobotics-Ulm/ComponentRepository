//--------------------------------------------------------------------------
//
//  Copyright (C) 2013 Matthias Lutz
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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

#ifndef _COMMUNICATION_HH
#define _COMMUNICATION_HH

#include "ace/INET_Addr.h"
#include "ace/SOCK_Connector.h"
#include "ace/SOCK_Acceptor.h"
#include "ace/SOCK_Stream.h"
#include "ace/SOCK_Dgram.h"
#include "ace/Event_Handler.h"
#include <string>
#include "ParameterStateStruct.hh"

class Communication : public ACE_Event_Handler, public SmartACE::TimerHandler
{
private:
	ACE_SOCK_Connector connector;
	ACE_SOCK_Stream peer;
	ACE_INET_Addr server_addr;
    ACE_INET_Addr peer_addr;
	bool connected;

	int connectionTimerId;
	bool connection_timeout;

	ParameterStateStruct::SettingsType localState;

public:

	int recv(std::string & msg);
	int send(std::string message);

	Communication();
	bool initConnection(bool connect=true);
	int connect();
	int disconnect();

	int handle_signal (int signum, siginfo_t *, ucontext_t *);
//	void timerExpired(const ACE_Time_Value & absolute_time, const void * arg);
    void timerExpired(const Smart::TimePoint &abs_time, const void * arg);

	~Communication();

};

#endif
