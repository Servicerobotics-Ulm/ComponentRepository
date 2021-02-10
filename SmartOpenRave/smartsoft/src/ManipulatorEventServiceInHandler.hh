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
#ifndef _MANIPULATOREVENTSERVICEINHANDLER_HH
#define _MANIPULATOREVENTSERVICEINHANDLER_HH

#include "ManipulatorEventServiceInHandlerCore.hh"

#include "SmartOpenRave.hh"

class ManipulatorEventServiceInHandler  : public ManipulatorEventServiceInHandlerCore
{		
public:
	ManipulatorEventServiceInHandler(Smart::InputSubject<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult,SmartACE::EventId>> *subject, const int &prescaleFactor=1);
	virtual ~ManipulatorEventServiceInHandler();
	
	virtual void on_ManipulatorEventServiceIn(const Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult,SmartACE::EventId> &input);
};

#endif
