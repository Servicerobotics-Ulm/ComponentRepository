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

#ifndef COMPONENTRMPBASESERVER_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTRMPBASESERVER_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentRMPBaseServerPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentRMPBaseServerOpcUaBackendPortFactory: public ComponentRMPBaseServerPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentRMPBaseServerOpcUaBackendPortFactory();
	virtual ~ComponentRMPBaseServerOpcUaBackendPortFactory();

	virtual void initialize(ComponentRMPBaseServer *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	
	virtual Smart::IPushServerPattern<CommBasicObjects::CommBaseState> * createBasePositionServer(const std::string &serviceName) override;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommBasePositionUpdate> * createBasePositionUpdateServer(const std::string &serviceName) override;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * createBaseQueryServer(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> * createBatteryEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>> batteryEventServerEventTestHandler) override;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * createNavigationVelocityServer(const std::string &serviceName) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTRMPBASESERVER_SERONET_SDK_PORTFACTORY_HH_ */
