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
#ifndef _LOCALIZATIONEVENTSERVICEOUT_EVENT_TEST_HANDLER_CORE_HH
#define _LOCALIZATIONEVENTSERVICEOUT_EVENT_TEST_HANDLER_CORE_HH
		
#include "aceSmartSoft.hh"

#include <CommLocalizationObjects/CommLocalizationEventParameter.hh>
#include <CommLocalizationObjects/CommLocalizationEventResult.hh>
#include <CommLocalizationObjects/LocalizationEventState.hh>

class LocalizationEventServiceOutEventTestHandlerCore : public SmartACE::EventTestHandler<CommLocalizationObjects::CommLocalizationEventParameter, CommLocalizationObjects::CommLocalizationEventResult, CommLocalizationObjects::LocalizationEventState>
{
};
#endif