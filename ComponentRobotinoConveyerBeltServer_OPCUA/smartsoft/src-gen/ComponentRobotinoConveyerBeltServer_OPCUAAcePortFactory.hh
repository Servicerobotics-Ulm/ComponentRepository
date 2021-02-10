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

#ifndef COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_ACE_PORTFACTORY_HH_
#define COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentRobotinoConveyerBeltServer_OPCUAImpl.hh"

// include the main component-definition class
#include "ComponentRobotinoConveyerBeltServer_OPCUAPortFactoryInterface.hh"

class ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory: public ComponentRobotinoConveyerBeltServer_OPCUAPortFactoryInterface
{
private:
	ComponentRobotinoConveyerBeltServer_OPCUAImpl *componentImpl;
public:
	ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory();
	virtual ~ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory();

	virtual void initialize(ComponentRobotinoConveyerBeltServer_OPCUA *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IEventClientPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult> * createCommDigitalInputEventIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * createCommIOValuesQueryServiceReq() override;
	virtual Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * createCommPowerOutputSendOut() override;
	
	virtual Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * createRobotinoConveyerBeltEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> robotinoConveyerBeltEventOutEventTestHandler) override;
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTROBOTINOCONVEYERBELTSERVER_OPCUA_ACE_PORTFACTORY_HH_ */
