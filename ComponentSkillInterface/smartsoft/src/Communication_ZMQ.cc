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
#include "Communication_ZMQ.hh"
#include <iostream>
#include <sstream>
#include "stdlib.h"
#include <string>
#include <fstream>



Communication_ZMQ::Communication_ZMQ():
	context(1),
	request_socket(context, ZMQ_REP),
	reply_socket(context, ZMQ_PUB)
{
 connected = false;
 verbose = false;
 port = 0;
}


bool Communication_ZMQ::setConfig(std::string ip, unsigned int port, bool use_socket_timeout, unsigned int timeout_value_sec, bool verbose){


	this->port = port;
	this->verbose = verbose;

	this->connect();


	return true;
}

int Communication_ZMQ::connect(){
	{
	std::stringstream ss;
	ss << "tcp://*:"<<port;
	request_socket.bind(ss.str());
	}

	{
	std::stringstream ss;
	ss << "tcp://*:"<<port+1;
	reply_socket.bind(ss.str());
	}

	int timeout_ms = 500;
	request_socket.setsockopt( ZMQ_RCVTIMEO, timeout_ms);



	return 0;
}

int Communication_ZMQ::disconnect(){

	return 0;
}

int Communication_ZMQ::recv(std::string & msg){

	zmq::message_t request;

	//  Wait for next request from client
	try{
		bool not_timeout = request_socket.recv (&request);

		if( not_timeout )
		{
			//  Send reply back to client TODO should be used to eval if the request is valid
			zmq::message_t ack;
			request_socket.send(ack);
		} else {
			return 1;
		}
	}
	catch(zmq::error_t& err)
	{
		std::cout<<"ELSE: Error reading from socket "<<std::endl;
		ACE_OS::sleep(ACE_Time_Value(0, 500000));
		return -1;
	}


	msg = std::string(static_cast<char*>(request.data()), request.size());

	return 0;
}


int Communication_ZMQ::recv_lines(std::deque<std::string> & deque){

	std::string message;
	int retval = this->recv(message);

	if(retval == 0){
		std::cout<<"[Communication_ZMQ]Got message: |"<<message<<"|"<<std::endl;
		deque.push_back(message);
	}

	return retval;
}


int Communication_ZMQ::send(const std::string& message){

	SmartACE::SmartRecursiveGuard g(writeComLock);
	std::cout << "[Communication_ZMQ::send] Message: |" << message << "|"<<std::endl;


	zmq::message_t reply(message.length());
	memcpy ((void *) reply.data (), message.c_str(), message.length());
	if(reply_socket.send(reply) == false){
		std::cout << "[Communication_ZMQ::send] Error on send!"<<std::endl;
		return -1;
	} else {
		std::cout<< "[Communication_ZMQ::send] Send OK!"<<std::endl;
		return 0;
	}
}


Communication_ZMQ::~Communication_ZMQ(){
	std::cout<<"Destructor Communication!"<<std::endl;
	this->disconnect();
}
