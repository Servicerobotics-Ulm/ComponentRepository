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

#ifndef SMARTPLANNERBREADTHFIRSTSEARCH_OPC_UA_BACKEND_PORTFACTORY_HH_
#define SMARTPLANNERBREADTHFIRSTSEARCH_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "SmartPlannerBreadthFirstSearchPortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class SmartPlannerBreadthFirstSearchOpcUaBackendPortFactory: public SmartPlannerBreadthFirstSearchPortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	SmartPlannerBreadthFirstSearchOpcUaBackendPortFactory();
	virtual ~SmartPlannerBreadthFirstSearchOpcUaBackendPortFactory();

	virtual void initialize(SmartPlannerBreadthFirstSearch *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateClient() override;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * createCurMapClient() override;
	
	virtual Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> * createCurrGridMapPushServiceOut(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState> * createPlannerEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState>> plannerEventServerEventTestHandler) override;
	virtual Smart::IPushServerPattern<CommNavigationObjects::CommPlannerGoal> * createPlannerGoalServer(const std::string &serviceName) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* SMARTPLANNERBREADTHFIRSTSEARCH_SERONET_SDK_PORTFACTORY_HH_ */
