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
#ifndef _MOBILEMANIPULATORSTATESERVICEIN_UPCALL_INTERFACE_HH
#define _MOBILEMANIPULATORSTATESERVICEIN_UPCALL_INTERFACE_HH

#include "CommManipulatorObjects/CommMobileManipulatorState.hh"

class MobileManipulatorStateServiceInUpcallInterface {
public:
	virtual ~MobileManipulatorStateServiceInUpcallInterface() {  }

	virtual void on_MobileManipulatorStateServiceIn(const CommManipulatorObjects::CommMobileManipulatorState &input) = 0;
};

#endif