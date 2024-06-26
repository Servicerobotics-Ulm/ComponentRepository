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
#ifndef _IOEVENTSERVER_EVENT_TEST_HANDLER_USER_HH
#define _IOEVENTSERVER_EVENT_TEST_HANDLER_USER_HH
		
#include "IoEventServerEventTestHandlerCore.hh"

class IoEventServerEventTestHandler : public IoEventServerEventTestHandlerCore
{
public:
	virtual bool testEvent(
		CommBasicObjects::CommDigitalInputEventParameter &p,
		CommBasicObjects::CommDigitalInputEventResult &r,
		const CommBasicObjects::CommDigitalInputEventState &s
	) throw();
};
#endif
