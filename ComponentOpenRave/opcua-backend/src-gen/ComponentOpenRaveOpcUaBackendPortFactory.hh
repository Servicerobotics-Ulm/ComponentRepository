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

#ifndef COMPONENTOPENRAVE_OPC_UA_BACKEND_PORTFACTORY_HH_
#define COMPONENTOPENRAVE_OPC_UA_BACKEND_PORTFACTORY_HH_

// include the main component-definition class
#include "ComponentOpenRavePortFactoryInterface.hh"

#include <thread>
#include <chrono>

// include SeRoNetSDK library
#include <SeRoNetSDK/SeRoNet/Utils/Task.hpp>
#include <SeRoNetSDK/SeRoNet/Utils/Component.hpp>

class ComponentOpenRaveOpcUaBackendPortFactory: public ComponentOpenRavePortFactoryInterface
{
private:
	// internal component instance
	SeRoNet::Utils::Component *componentImpl;
	
	// component thread
	std::thread component_thread;
	
	// internal component thread method
	int task_execution();
public:
	ComponentOpenRaveOpcUaBackendPortFactory();
	virtual ~ComponentOpenRaveOpcUaBackendPortFactory();

	virtual void initialize(ComponentOpenRave *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> * createEnvironmentQueryServiceReq() override;
	virtual Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> * createGripperEventServiceIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommGripperState> * createGripperStateQueryServiceReq() override;
	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommGripperState> * createGripperStateServiceIn() override;
	virtual Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> * createManipulatorEventServiceIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * createMobileManipulatorStateQueryServiceReq() override;
	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> * createMobileManipulatorStateServiceIn() override;
	virtual Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectQueryServiceReq() override;
	virtual Smart::ISendClientPattern<CommManipulatorObjects::CommManipulatorTrajectory> * createSendTrajectoryOut() override;
	
	virtual Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectQueryServiceAnsw(const std::string &serviceName) override;
	virtual Smart::IEventServerPattern<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState> * createPlanningEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState>> planningEventServiceOutEventTestHandler) override;
	
	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTOPENRAVE_SERONET_SDK_PORTFACTORY_HH_ */
