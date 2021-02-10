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

#ifndef COMPONENTOPENRAVE_PORTFACTORYINTERFACE_HH_
#define COMPONENTOPENRAVE_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommManipulatorObjects/CommGripperState.hh>
#include <CommManipulatorObjects/CommGripperStateACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventParameter.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventParameterACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventResult.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventResultACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventState.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameter.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameterACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventResult.hh>
#include <CommManipulatorObjects/CommManipulatorEventResultACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventState.hh>
#include <CommManipulatorObjects/CommManipulatorEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectoryACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironment.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironmentACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionId.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionIdACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectPropertiesACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentOpenRave.hh"

// forward declaration
class ComponentOpenRave;

class ComponentOpenRavePortFactoryInterface {
public:
	ComponentOpenRavePortFactoryInterface() { };
	virtual ~ComponentOpenRavePortFactoryInterface() { };

	virtual void initialize(ComponentOpenRave *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> * createEnvironmentQueryServiceReq() = 0;
	virtual Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> * createGripperEventServiceIn() = 0;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommGripperState> * createGripperStateQueryServiceReq() = 0;
	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommGripperState> * createGripperStateServiceIn() = 0;
	virtual Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> * createManipulatorEventServiceIn() = 0;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * createMobileManipulatorStateQueryServiceReq() = 0;
	virtual Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> * createMobileManipulatorStateServiceIn() = 0;
	virtual Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectQueryServiceReq() = 0;
	virtual Smart::ISendClientPattern<CommManipulatorObjects::CommManipulatorTrajectory> * createSendTrajectoryOut() = 0;
	
	virtual Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * createObjectQueryServiceAnsw(const std::string &serviceName) = 0;
	virtual Smart::IEventServerPattern<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState> * createPlanningEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState>> planningEventServiceOutEventTestHandler) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTOPENRAVE_PORTFACTORYINTERFACE_HH_ */
