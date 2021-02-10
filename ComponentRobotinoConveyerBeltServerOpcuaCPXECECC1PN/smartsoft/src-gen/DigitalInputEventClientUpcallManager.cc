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
#include "DigitalInputEventClientUpcallManager.hh"

DigitalInputEventClientUpcallManager::DigitalInputEventClientUpcallManager(
	Smart::InputSubject<Smart::EventInputType<CommRobotinoObjects::CommDigitalInputEventResult>> *subject,
	const int &prescaleFactor)
	:	Smart::IInputHandler<Smart::EventInputType<CommRobotinoObjects::CommDigitalInputEventResult>>(subject, prescaleFactor)
{  }
DigitalInputEventClientUpcallManager::~DigitalInputEventClientUpcallManager()
{  }

void DigitalInputEventClientUpcallManager::notify_upcalls(const Smart::EventInputType<CommRobotinoObjects::CommDigitalInputEventResult> &input)
{
	for(auto it=upcalls.begin(); it!=upcalls.end(); it++) {
		(*it)->on_digitalInputEventClient(input);
	}
}

void DigitalInputEventClientUpcallManager::attach(DigitalInputEventClientUpcallInterface *upcall)
{
	upcalls.push_back(upcall);
}
void DigitalInputEventClientUpcallManager::detach(DigitalInputEventClientUpcallInterface *upcall)
{
	upcalls.remove(upcall);
}