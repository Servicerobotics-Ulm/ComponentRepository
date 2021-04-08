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

#ifndef COMPONENTWEBOTSCONVEYORBELTOPCUA_PORTFACTORYINTERFACE_HH_
#define COMPONENTWEBOTSCONVEYORBELTOPCUA_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventParameter.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventParameterACE.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventResult.hh>
#include <CommRobotinoObjects/CommRobotinoConveyerBeltEventResultACE.hh>
#include <CommBasicObjects/CommTrafficLights.hh>
#include <CommBasicObjects/CommTrafficLightsACE.hh>
#include <CommRobotinoObjects/RobotinoConveyerBeltEventState.hh>
#include <CommRobotinoObjects/RobotinoConveyerBeltEventStateACE.hh>

#include <chrono>

// include component's main class
#include "ComponentWebotsConveyorBeltOpcua.hh"

// forward declaration
class ComponentWebotsConveyorBeltOpcua;

class ComponentWebotsConveyorBeltOpcuaPortFactoryInterface {
public:
	ComponentWebotsConveyorBeltOpcuaPortFactoryInterface() { };
	virtual ~ComponentWebotsConveyorBeltOpcuaPortFactoryInterface() { };

	virtual void initialize(ComponentWebotsConveyorBeltOpcua *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommTrafficLights> * createTrafficLightsServiceIn() = 0;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * createRobotinoConveyerBeltEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> robotinoConveyerBeltEventOutEventTestHandler) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTWEBOTSCONVEYORBELTOPCUA_PORTFACTORYINTERFACE_HH_ */
