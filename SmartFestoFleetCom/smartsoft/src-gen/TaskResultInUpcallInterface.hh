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
#ifndef _TASKRESULTIN_UPCALL_INTERFACE_HH
#define _TASKRESULTIN_UPCALL_INTERFACE_HH

#include "CommBasicObjects/CommTaskMessage.hh"

class TaskResultInUpcallInterface {
public:
	virtual ~TaskResultInUpcallInterface() {  }

	virtual void on_TaskResultIn(const CommBasicObjects::CommTaskMessage &input) = 0;
};

#endif
