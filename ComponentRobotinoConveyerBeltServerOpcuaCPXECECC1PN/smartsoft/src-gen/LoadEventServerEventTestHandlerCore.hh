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
#ifndef _LOADEVENTSERVER_EVENT_TEST_HANDLER_CORE_HH
#define _LOADEVENTSERVER_EVENT_TEST_HANDLER_CORE_HH
		
#include "aceSmartSoft.hh"

#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventParameter.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventResult.hh>
#include <CommRobotinoObjects/RobotinoConveyerBeltEventState.hh>

class LoadEventServerEventTestHandlerCore : public SmartACE::EventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>
{
};
#endif
