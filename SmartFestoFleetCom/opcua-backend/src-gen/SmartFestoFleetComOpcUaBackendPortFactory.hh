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

#ifndef SMARTFESTOFLEETCOM_OPC_UA_BACKEND_PORTFACTORY_HH_
#define SMARTFESTOFLEETCOM_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "SmartFestoFleetComPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class SmartFestoFleetComOpcUaBackendPortFactory: public SmartFestoFleetComPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	SmartFestoFleetComOpcUaBackendPortFactory();
	virtual ~SmartFestoFleetComOpcUaBackendPortFactory();

	virtual void initialize(SmartFestoFleetCom *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IEventClientPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField> * createLaserSafetyEventServiceIn() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommTaskMessage> * createTaskResultIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * createKbQueryClient() override;
	
	virtual Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> * createTaskEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> taskEventOutEventTestHandler) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* SMARTFESTOFLEETCOM_SERONET_SDK_PORTFACTORY_HH_ */
