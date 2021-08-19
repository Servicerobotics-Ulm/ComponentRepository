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

#ifndef SMARTJOBDISPATCHER_OPC_UA_BACKEND_PORTFACTORY_HH_
#define SMARTJOBDISPATCHER_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "SmartJobDispatcherPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class SmartJobDispatcherOpcUaBackendPortFactory: public SmartJobDispatcherPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	SmartJobDispatcherOpcUaBackendPortFactory();
	virtual ~SmartJobDispatcherOpcUaBackendPortFactory();

	virtual void initialize(SmartJobDispatcher *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * createCommKBQueryReq() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommNavigationObjects::CommNavigationTopologyMap> * createNavigationTopologyMapQueryReq() override;
	virtual Smart::IQueryClientPattern<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan> * createSymbolicPannerServiceReq() override;
	virtual Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * createJobSource2EventClient() override;
	virtual Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * createJobSource3EventClient() override;
	virtual Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * createJobSourceEventClient() override;
	
	virtual Smart::IPushServerPattern<CommBasicObjects::CommTaskMessage> * createTaskResultOut(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> * createJobEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> jobEventServerEventTestHandler) override;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommTaskMessage> * createJobResultSendServer(const std::string &serviceName) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* SMARTJOBDISPATCHER_SERONET_SDK_PORTFACTORY_HH_ */
