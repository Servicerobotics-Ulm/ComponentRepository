//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "CommandResultTask.hh"
#include "SmartFestoFleetCom.hh"

#include <iostream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

// trim from start
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}


bool IsNonContentChar(char c)
{
	switch(c)
	{
	case '(':
	case ')':
	//case '"':
		return true;
	default:
		return false;

	}
}

bool IsSlashChar(char c)
{
	switch(c)
	{
	case '\\':
		return true;
	default:
		return false;

	}
}

bool CommandResultTask::parseLispString(std::string msg, std::string& tag, std::string& value)
{
	std::cout<<"Got Param from Lisp: "<<msg<<std::endl;
	ltrim(msg);
	msg.erase(std::remove_if(msg.begin(),msg.end(), &IsNonContentChar),msg.end());
	std::string blank(" ");

	std::size_t found = msg.find(blank);
	if (found!=std::string::npos)
	{
		tag = msg.substr(0, found);
                value = msg.substr(found+1, msg.length());
		return true;
	} else
	{
		return false;
	}

}

CommandResultTask::CommandResultTask(SmartACE::SmartComponent *comp) 
:	CommandResultTaskCore(comp)
{
	std::cout << "constructor CommandResultTask\n";
}
CommandResultTask::~CommandResultTask() 
{
	std::cout << "destructor CommandResultTask\n";
}



int CommandResultTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int CommandResultTask::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	Smart::StatusCode status;
	CommBasicObjects::CommTaskMessage r;
	status = COMP->taskResultIn->getUpdateWait(r);

	if(status != Smart::SMART_OK){
		std::cout<<"Error on sequencerNewestClient->getUpdateWait(r): "<<Smart::StatusCodeConversion(status)<<std::endl;
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		return 0;
	}
	std::string msg = r.getMsg();
	msg.erase(0,1); //remove front \"
	msg.erase(msg.size()-1,1); //remove tail \"
	msg.erase(std::remove_if(msg.begin(),msg.end(), &IsSlashChar),msg.end());
	std::cout<<"Send to sock:|"<<msg<<"\n|"<<std::endl;
	COMP->writeComLock.acquire();
		COMP->com.send(msg + "\n");
	COMP->writeComLock.release();
//	std::string tag,value;
//	if(parseLispString(msg,tag,value)){
//
//		if (tag == "SYSTEMSTATE"){
//			COMP->statusDataLock.acquire();
//				COMP->statusData = value;
//			COMP->statusDataLock.release();
//		} else if(tag == "PROGRESS"){
//			COMP->statusDataLock.acquire();
//				COMP->jobProgress = value;
//			COMP->statusDataLock.release();
//		}else{
//			std::cout<<"[ParameterHandler::handleSend] ERROR unknown parameter: "<<msg<<std::endl;
//			std::cout<<"Tag :"<<tag<<"END"<<std::endl;
//		}
//	} else {
//		std::cout<<"[ParameterHandler::handleSend] ERROR parsing lisp string: "<<msg<<std::endl;
//	}

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int CommandResultTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
