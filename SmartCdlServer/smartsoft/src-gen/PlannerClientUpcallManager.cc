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
#include "PlannerClientUpcallManager.hh"

PlannerClientUpcallManager::PlannerClientUpcallManager(
	Smart::InputSubject<CommNavigationObjects::CommPlannerGoal> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<CommNavigationObjects::CommPlannerGoal>(subject, prescaleFactor)
{  }
PlannerClientUpcallManager::~PlannerClientUpcallManager()
{  }

void PlannerClientUpcallManager::notify_upcalls(const CommNavigationObjects::CommPlannerGoal &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_PlannerClient(input);
	}
}

void PlannerClientUpcallManager::attach(PlannerClientUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void PlannerClientUpcallManager::detach(PlannerClientUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
