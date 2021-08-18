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

#ifndef COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentRobotinoConveyerBeltServer_OPCUAPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentRobotinoConveyerBeltServer_OPCUAOpcUaBackendPortFactory: public ComponentRobotinoConveyerBeltServer_OPCUAPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentRobotinoConveyerBeltServer_OPCUAOpcUaBackendPortFactory();
	virtual ~ComponentRobotinoConveyerBeltServer_OPCUAOpcUaBackendPortFactory();

	virtual void initialize(ComponentRobotinoConveyerBeltServer_OPCUA *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IEventClientPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult> * createCommDigitalInputEventIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * createCommIOValuesQueryServiceReq() override;
	virtual Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * createCommPowerOutputSendOut() override;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * createRobotinoConveyerBeltEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> robotinoConveyerBeltEventOutEventTestHandler) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_SERONET_SDK_PORTFACTORY_HH_ */
