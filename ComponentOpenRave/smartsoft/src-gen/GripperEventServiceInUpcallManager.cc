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
#include "GripperEventServiceInUpcallManager.hh"

GripperEventServiceInUpcallManager::GripperEventServiceInUpcallManager(
	Smart::InputSubject<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>>(subject, prescaleFactor)
{  }
GripperEventServiceInUpcallManager::~GripperEventServiceInUpcallManager()
{  }

void GripperEventServiceInUpcallManager::notify_upcalls(const Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult> &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_GripperEventServiceIn(input);
	}
}

void GripperEventServiceInUpcallManager::attach(GripperEventServiceInUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void GripperEventServiceInUpcallManager::detach(GripperEventServiceInUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
