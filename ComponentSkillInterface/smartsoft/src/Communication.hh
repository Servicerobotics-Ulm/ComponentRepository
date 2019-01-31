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

#ifndef _COMMUNICATION_HH
#define _COMMUNICATION_HH

#include <string>

class Communication
{

public:

	virtual int recv_lines(std::deque<std::string> & deque) = 0;
	virtual int send(const std::string& message) = 0;

	Communication(){}
	virtual ~Communication() {}
	virtual bool setConfig(std::string ip, unsigned int port, bool use_socket_timeout, unsigned int timeout_value_sec, bool verbose = false) = 0;
	virtual int connect()= 0;
	virtual int disconnect()= 0;
};

#endif
