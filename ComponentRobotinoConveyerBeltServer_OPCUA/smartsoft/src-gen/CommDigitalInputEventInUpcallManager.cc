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
#include "CommDigitalInputEventInUpcallManager.hh"

CommDigitalInputEventInUpcallManager::CommDigitalInputEventInUpcallManager(
	Smart::InputSubject<Smart::EventInputType<CommBasicObjects::CommDigitalInputEventResult>> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<Smart::EventInputType<CommBasicObjects::CommDigitalInputEventResult>>(subject, prescaleFactor)
{  }
CommDigitalInputEventInUpcallManager::~CommDigitalInputEventInUpcallManager()
{  }

void CommDigitalInputEventInUpcallManager::notify_upcalls(const Smart::EventInputType<CommBasicObjects::CommDigitalInputEventResult> &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_CommDigitalInputEventIn(input);
	}
}

void CommDigitalInputEventInUpcallManager::attach(CommDigitalInputEventInUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void CommDigitalInputEventInUpcallManager::detach(CommDigitalInputEventInUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}