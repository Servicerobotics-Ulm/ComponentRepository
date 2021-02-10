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

#ifndef COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNOpcUaBackendPortFactory: public ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNOpcUaBackendPortFactory();
	virtual ~ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNOpcUaBackendPortFactory();

	virtual void initialize(ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IEventClientPattern<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult> * createDigitalInputEventClient() override;
	virtual Smart::IQueryClientPattern<CommRobotinoObjects::CommRobotinoIOValues, CommRobotinoObjects::CommRobotinoIOValues> * createIOQueryClient() override;
	virtual Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * createPowerOutPutSendClient() override;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * createLoadEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> loadEventServerEventTestHandler) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTROBOTINOCONVEYERBELTSERVEROPCUACPXECECC1PN_SERONET_SDK_PORTFACTORY_HH_ */