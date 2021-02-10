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

#ifndef COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_PORTFACTORYINTERFACE_HH_
#define COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommRobotinoObjects/CommDigitalInputEventParameter.hh>
#include <CommRobotinoObjects/CommDigitalInputEventParameterACE.hh>
#include <CommRobotinoObjects/CommDigitalInputEventResult.hh>
#include <CommRobotinoObjects/CommDigitalInputEventResultACE.hh>
#include <CommRobotinoObjects/CommDigitalInputEventState.hh>
#include <CommRobotinoObjects/CommDigitalInputEventStateACE.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventParameter.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventParameterACE.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventResult.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventResultACE.hh>
#include <CommRobotinoObjects/CommRobotinoIOValues.hh>
#include <CommRobotinoObjects/CommRobotinoIOValuesACE.hh>
#include <CommRobotinoObjects/CommRobotinoPowerOutputValue.hh>
#include <CommRobotinoObjects/CommRobotinoPowerOutputValueACE.hh>
#include <CommRobotinoObjects/RobotinoConveyerBeltEventState.hh>
#include <CommRobotinoObjects/RobotinoConveyerBeltEventStateACE.hh>

#include <chrono>

// include component's main class
#include "ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN.hh"

// forward declaration
class ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN;

class ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNPortFactoryInterface {
public:
	ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNPortFactoryInterface() { };
	virtual ~ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNPortFactoryInterface() { };

	virtual void initialize(ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IEventClientPattern<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult> * createDigitalInputEventClient() = 0;
	virtual Smart::IQueryClientPattern<CommRobotinoObjects::CommRobotinoIOValues, CommRobotinoObjects::CommRobotinoIOValues> * createIOQueryClient() = 0;
	virtual Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * createPowerOutPutSendClient() = 0;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * createLoadEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> loadEventServerEventTestHandler) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_PORTFACTORYINTERFACE_HH_ */
