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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#include "TaskResultInUpcallManager.hh"

TaskResultInUpcallManager::TaskResultInUpcallManager(
	Smart::InputSubject<CommBasicObjects::CommTaskMessage> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<CommBasicObjects::CommTaskMessage>(subject, prescaleFactor)
{  }
TaskResultInUpcallManager::~TaskResultInUpcallManager()
{  }

void TaskResultInUpcallManager::notify_upcalls(const CommBasicObjects::CommTaskMessage &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_TaskResultIn(input);
	}
}

void TaskResultInUpcallManager::attach(TaskResultInUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void TaskResultInUpcallManager::detach(TaskResultInUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
