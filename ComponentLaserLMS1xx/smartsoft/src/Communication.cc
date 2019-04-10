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

#include "Communication.hh"
#include "ComponentLaserLMS1xx.hh"
#include <iostream>
#include <sstream>
#include "stdlib.h"
#include <string>
#include <fstream>


Communication::Communication(){
 connected = false;
 ip = "";
 port = 0;
}


bool Communication::initConnection(const std::string& ip, const unsigned int& port){

	this->ip = ip;
	this->port = port;

	 // set this class as default handler for SIGPIPE signal
	COMP->getComponentImpl()->getReactorTask()->getImpl()->register_handler(SIGPIPE, this);

	int ret;
	do{
		ret = this->connect();
		ACE_OS::sleep(ACE_Time_Value(0, 500000));
	}while(ret != 0);

	return true;
}

int Communication::connect(){
	std::stringstream ss;
	ss << ip << ":"<< port;
	std::string addressString(ss.str());
	std::cout<<"Connecting to : "<<addressString<<std::endl;
	if(server_addr.set (addressString.c_str()) == -1){
		std::cerr<<"Error on set!"<<std::endl;
                connected = false;
		return 1;
	}
	if(connector.connect(peer,server_addr)== -1){
		std::cerr<<"Error on connect!"<<std::endl;
                connected = false;
		return -1;
	}else{
		std::cout<<"Successfully connected to FleetCom!"<<std::endl;
        	connected = true;
	}
	return 0;
}

int Communication::disconnect(){
        connected = false;
	return peer.close();
}

bool Communication::isConnected(){
	return connected;
}

int Communication::recv(std::string & msg){
	char buffer[2048];
	int messageSize;

	memset(buffer,0,sizeof(buffer));
	ACE_Time_Value timeout(1);
	if(connected){
		messageSize = peer.recv(buffer, sizeof(buffer), &timeout);
		if(messageSize <0){
			if(errno == ETIME) {
					// timeout is used to prevent the task from blocking
        	                return 1;
			} 
			else {
    				std::cout<<"ELSE: Error reading from socket errno: "<<errno<<std::endl;
					ACE_OS::sleep(ACE_Time_Value(0, 500000));
				return -1;
    			}
		}else if (messageSize==0){
			std::cout<<"ERROR: EOF --> connection closed!"<<std::endl;
	                this->disconnect();
			return -1;
		}
	} else {
		this->connect();
		ACE_OS::sleep(ACE_Time_Value(0, 500000));
		return -1;
	}
	
	msg = &buffer[0];
	return 0;
}

int Communication::recv(void *buf, size_t len){
	//char buffer[len];
	int messageSize = -1;

	ACE_Time_Value timeout(1);
	if(connected){
		messageSize = peer.recv(buf, len, &timeout);
		if(messageSize <0){
			if(errno == ETIME) {
					// timeout is used to prevent the task from blocking
        	        return messageSize;
			}
			else {
    				std::cout<<"ELSE: Error reading from socket errno: "<<errno<<std::endl;
					ACE_OS::sleep(ACE_Time_Value(0, 500000));
				return messageSize;
			}
		}
	} else {
		this->connect();
		ACE_OS::sleep(ACE_Time_Value(0, 500000));
		return messageSize;
	}

	return messageSize;
}


int Communication::send(std::string message){

//		std::cout<<"=================================="<<std::endl;
//		std::cout << "Message: '" << message << "'";

		if(peer.send_n(message.c_str(), message.length()) == -1){
			std::cout << "[Communication::send] Error on send_n!";
			return -1;
		} else{
			//std::cout<< "Send successful!"<<std::endl;
			return 0;
		}
}

int Communication::send(const void *buf, size_t len){

//		std::cout<<"=================================="<<std::endl;
//		std::cout << "Message: '" << message << "'";

		if(peer.send_n(buf, len) == -1){
			std::cout << "[Communication::send] Error on send_n!";
			return -1;
		} else{
			//std::cout<< "Send successful!"<<std::endl;
			return 0;
		}
}

int Communication::handle_signal (int signum, siginfo_t *, ucontext_t *){
	this->disconnect();
	std::cout<<"Communication::handle_signal: SIGPIPE --> stop both tasks."<<std::endl;
	return 0;
}


Communication::~Communication(){
	std::cout<<"Destructor Communication!"<<std::endl;
//	this->disconnect();

}
