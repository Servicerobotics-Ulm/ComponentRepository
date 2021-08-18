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
#include "SmartFestoFleetCom.hh"
#include <iostream>
#include <sstream>
#include "stdlib.h"
#include <string>
#include <fstream>


Communication::Communication(){
 connected = false;
 connectionTimerId = -1; //reset timer
 connection_timeout = false;
}

bool Communication::initConnection(bool connect){

	localState = COMP->getGlobalState().getSettings();
	 // set this class as default handler for SIGPIPE signal
	COMP->getComponentImpl()->getReactorTask()->getImpl()->register_handler(SIGPIPE, this);

	if(connect){
		int ret;
		do{
			ret = this->connect();
			ACE_OS::sleep(ACE_Time_Value(0, 500000));
		}while(ret != 0);
	}
	return true;
}

int Communication::connect(){
	std::string addressString =  localState.getIp() + ":" + localState.getPort();
	std::cout<<"[Communication::connect()] Connecting to : "<<addressString<<std::endl;
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
		std::cout<<"[Communication::connect()] Successfully connected ..!"<<std::endl;
        	connected = true;
        	if(localState.getUse_socket_timeout() && connectionTimerId == -1){
				connection_timeout = false;

				std::chrono::seconds sec(localState.getSocket_timeout_s());
				std::chrono::milliseconds msec(0);

				std::cout << "[Communication::connect()]  scheduleTimer relative time: " << localState.getSocket_timeout_s() << " : " << 0 << std::endl;
				connectionTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL, sec+msec);
			}
	}
	return 0;
}

int Communication::disconnect(){
	connected = false;
	if(localState.getUse_socket_timeout() && connectionTimerId != -1)
	{
		std::cout << "[Communication::disconnect()]  cancelTimer!"<< std::endl;
		COMP->getComponentImpl()->getTimerManager()->cancelTimer(connectionTimerId);
		connectionTimerId = -1;
	}
	return peer.close();
}

int Communication::recv(std::string & msg){
	char buffer[2048];
	int messageSize;

	memset(buffer,0,sizeof(buffer));
	ACE_Time_Value timeout(0,500000);
	if(connected){
		messageSize = peer.recv(buffer, sizeof(buffer), &timeout);

		if(messageSize <0){
			if(errno == ETIME) {
				// timeout is used to prevent the task from blocking

				if(connection_timeout == true && localState.getUse_socket_timeout()){
					//no data send for long time close connection!
					this->disconnect();
					this->connect();
				}

				return 1;

			}
			else {
				std::cout<<"ELSE: Error reading from socket errno: "<<errno<<std::endl;
				std::cout<<" --> close connection!"<<std::endl;
				this->disconnect();
				ACE_OS::sleep(ACE_Time_Value(0, 500000));
				return -1;
			}

		} else if (messageSize==0){
			std::cout<<"ERROR: EOF --> connection closed!"<<std::endl;
			this->disconnect();
			return -1;
		}


		if(localState.getUse_socket_timeout() == true && connectionTimerId != -1)
		{

			if(localState.getVerbose()){
				std::cout << "[Communication::recv()]  rescheduleTimer!"<< std::endl;
			}
			COMP->getComponentImpl()->getTimerManager()->cancelTimer(connectionTimerId);
			connectionTimerId = -1;
			std::chrono::seconds sec(localState.getSocket_timeout_s());
			std::chrono::milliseconds msec(0);

			connectionTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,NULL, sec+msec);
            connection_timeout = false;
		}

		msg = &buffer[0];
		return 0;

	} else {
		this->connect();
		ACE_OS::sleep(ACE_Time_Value(0, 500000));
		return -1;
	}

}


int Communication::send(std::string message){

//		std::cout<<"=================================="<<std::endl;
		std::cout << "[Communication::send] Message: |" << message << "|";

		if(peer.send_n(message.c_str(), message.length()) == -1){
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



//void Communication::timerExpired(const ACE_Time_Value & absolute_time, const void * arg)
void Communication::timerExpired(const Smart::TimePoint &abs_time, const void * arg)
{
	std::cout<<"[Communication:timerExpired] ERROR TIMEOUT on socket connection!"<<std::endl;
	connection_timeout = true;
}

Communication::~Communication(){
	std::cout<<"Destructor Communication!"<<std::endl;
	this->disconnect();
}

