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

#include "Communication_sock.hh"
#include "ComponentSkillInterface.hh"
#include <iostream>
#include <sstream>
#include "stdlib.h"
#include <string>
#include <fstream>


Communication_sock::Communication_sock(){
 connected = false;
 connectionTimerId = -1; //reset timer
 connection_timeout = false;
 verbose = false;
 port = 0;
 timeout_value_sec = 2;
 use_socket_timeout = false;
}


bool Communication_sock::setConfig(std::string ip, unsigned int port, bool use_socket_timeout, unsigned int timeout_value_sec, bool verbose){

	this->ip=ip;
	this->port=port;
	this->use_socket_timeout = use_socket_timeout;
	this->timeout_value_sec = timeout_value_sec;
	this->verbose = verbose;

	 // set this class as default handler for SIGPIPE signal
	COMP->getComponentImpl()->getReactorTask()->getImpl()->register_handler(SIGPIPE, this);

	return true;

}

int Communication_sock::connect(){

	std::stringstream ss;
	ss<<this->ip + ":"<<this->port;
	std::string addressString =  ss.str();
	std::cout<<"[Communication_sock::connect()] Connecting to : "<<addressString<<std::endl;
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
		std::cout<<"[Communication_sock::connect()] Successfully connected ..!"<<std::endl;
        	connected = true;
        	if(this->use_socket_timeout && connectionTimerId == -1){
				connection_timeout = false;
				int sec, msec;
				msec = 0;
				sec = this->timeout_value_sec;
				std::cout << "[Communication_sock::connect()]  scheduleTimer relative time: " << sec << " : " << msec << std::endl;
				connectionTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,(void*)NULL, SmartACE::convertToStdDurFrom(ACE_Time_Value( sec, msec*1000)));
			}
	}
	return 0;
}

int Communication_sock::disconnect(){


	connected = false;
	if(this->use_socket_timeout && connectionTimerId != -1)
	{
		std::cout << "[Communication_sock::disconnect()]  cancelTimer!"<< std::endl;
		COMP->getComponentImpl()->getTimerManager()->cancelTimer(connectionTimerId);
		connectionTimerId = -1;
	}
	return peer.close();
}

int Communication_sock::recv(std::string & msg){

	char buffer[2048];
	int messageSize;

	memset(buffer,0,sizeof(buffer));
	ACE_Time_Value timeout(0,500000);
	if(connected){
		messageSize = peer.recv(buffer, sizeof(buffer), &timeout);

		if(messageSize <0){
			if(errno == ETIME) {
				// timeout is used to prevent the task from blocking

				if(connection_timeout == true && this->use_socket_timeout){
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


		if(this->use_socket_timeout == true && connectionTimerId != -1)
		{

			if(this->verbose){
				std::cout << "[Communication_sock::recv()]  rescheduleTimer!"<< std::endl;
			}
			COMP->getComponentImpl()->getTimerManager()->cancelTimer(connectionTimerId);
			connectionTimerId = -1;
			int sec, msec;
			msec = 0;
			sec = this->timeout_value_sec;
			connectionTimerId = COMP->getComponentImpl()->getTimerManager()->scheduleTimer(this,(void*)NULL, SmartACE::convertToStdDurFrom(ACE_Time_Value( sec, msec*1000)));
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


int Communication_sock::recv_lines(std::deque<std::string> & deque){

	std::stringstream messageStream;
	std::string message;
	int retval = this->recv(message);

	if(retval == 0){

		messageStream << messageRest;
		std::cout<<"[Communication_sock]Got message: |"<<message<<"|"<<std::endl;
		messageStream << message;

		//std::cout<<"MStream|"<<messageStream.str() <<"|"<<std::endl;
		messageRest = splitIntoLines(deque, messageStream);

		removeMessagesWithNonePrintChars(deque);

	}

	return retval;
}


int Communication_sock::send(const std::string& message){

//	std::cout<<"Wait for lock"<<std::endl;
		SmartACE::SmartRecursiveGuard g(writeComLock);

//		std::cout<<"=================================="<<std::endl;
		std::cout << "[Communication_sock::send] Message: |" << message << "|";

		if(peer.send_n(message.c_str(), message.length()) == -1){
			std::cout << "[Communication_sock::send] Error on send_n!";
			return -1;
		} else{
			//std::cout<< "Send successful!"<<std::endl;
			return 0;
		}
}



void Communication_sock::removeMessagesWithNonePrintChars(std::deque <std::string> &mydeque){
std::deque<std::string>::iterator it2;
for(it2=mydeque.begin(); it2!=mydeque.end();)
{
	std::string::iterator it = find_if(it2->begin(),it2->end(), [] (const char& c) { 	if (std::isprint(c) == 0) return true; else	return false;});
	if(it != it2->end()){
		std::cout<<"ERROR the message does contains control chars --> remove MSG!"<<std::endl;
		it2 = mydeque.erase(it2);
	} else {
		++it2;
	}
}
}

std::string Communication_sock::splitIntoLines(std::deque <std::string> &result, std::stringstream &ss)
{
	std::string line;
	std::string rest = "";
	do{
		line = "";
		std::getline(ss,line);
		//std::cout << "getline: line: "<<line<<" good: " << ss.good() << " eof: " << ss.eof() << " fail: " << ss.fail() << " bad: " << ss.bad() << std::endl;
		if(ss.good()){
			result.push_back( line );
		}else
		{
			rest = line;
		}
	}
	while (ss.good());
	return rest;
}




int Communication_sock::handle_signal (int signum, siginfo_t *, ucontext_t *){
	this->disconnect();
	std::cout<<"Communication_sock::handle_signal: SIGPIPE --> stop both tasks."<<std::endl;
	return 0;
}


void Communication_sock::timerExpired(const Smart::TimePoint &abs_time, const void * arg)
{
	std::cout<<"[Communication_sock:timerExpired] ERROR TIMEOUT on socket connection!"<<std::endl;
	connection_timeout = true;
}

Communication_sock::~Communication_sock(){
	std::cout<<"Destructor Communication_sock!"<<std::endl;
	this->disconnect();
}
