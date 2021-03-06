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
#include "PlannerGoalPushClientUpcallManager.hh"

PlannerGoalPushClientUpcallManager::PlannerGoalPushClientUpcallManager(
	Smart::InputSubject<CommNavigationObjects::CommPlannerGoal> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<CommNavigationObjects::CommPlannerGoal>(subject, prescaleFactor)
{  }
PlannerGoalPushClientUpcallManager::~PlannerGoalPushClientUpcallManager()
{  }

void PlannerGoalPushClientUpcallManager::notify_upcalls(const CommNavigationObjects::CommPlannerGoal &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_plannerGoalPushClient(input);
	}
}

void PlannerGoalPushClientUpcallManager::attach(PlannerGoalPushClientUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void PlannerGoalPushClientUpcallManager::detach(PlannerGoalPushClientUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
