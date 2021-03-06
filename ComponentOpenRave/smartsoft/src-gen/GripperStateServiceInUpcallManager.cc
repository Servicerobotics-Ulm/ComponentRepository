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
#include "GripperStateServiceInUpcallManager.hh"

GripperStateServiceInUpcallManager::GripperStateServiceInUpcallManager(
	Smart::InputSubject<CommManipulatorObjects::CommGripperState> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<CommManipulatorObjects::CommGripperState>(subject, prescaleFactor)
{  }
GripperStateServiceInUpcallManager::~GripperStateServiceInUpcallManager()
{  }

void GripperStateServiceInUpcallManager::notify_upcalls(const CommManipulatorObjects::CommGripperState &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_GripperStateServiceIn(input);
	}
}

void GripperStateServiceInUpcallManager::attach(GripperStateServiceInUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void GripperStateServiceInUpcallManager::detach(GripperStateServiceInUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}
