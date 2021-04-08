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

#ifndef COMPONENTWEBOTSURSERVER_PORTFACTORYINTERFACE_HH_
#define COMPONENTWEBOTSURSERVER_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommDigitalInputEventParameter.hh>
#include <CommBasicObjects/CommDigitalInputEventParameterACE.hh>
#include <CommBasicObjects/CommDigitalInputEventResult.hh>
#include <CommBasicObjects/CommDigitalInputEventResultACE.hh>
#include <CommBasicObjects/CommDigitalInputEventState.hh>
#include <CommBasicObjects/CommDigitalInputEventStateACE.hh>
#include <CommBasicObjects/CommIOValues.hh>
#include <CommBasicObjects/CommIOValuesACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameter.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameterACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventResult.hh>
#include <CommManipulatorObjects/CommManipulatorEventResultACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventState.hh>
#include <CommManipulatorObjects/CommManipulatorEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectoryACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorPrograms.hh>
#include <CommManipulatorObjects/CommMobileManipulatorProgramsACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

#include <chrono>

// include component's main class
#include "ComponentWebotsURServer.hh"

// forward declaration
class ComponentWebotsURServer;

class ComponentWebotsURServerPortFactoryInterface {
public:
	ComponentWebotsURServerPortFactoryInterface() { };
	virtual ~ComponentWebotsURServerPortFactoryInterface() { };

	virtual void initialize(ComponentWebotsURServer *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateServiceIn() = 0;
	
	virtual Smart::IEventServerPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState> * createIoEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>> ioEventServerEventTestHandler) = 0;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * createIoQueryServer(const std::string &serviceName) = 0;
	virtual Smart::IEventServerPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState> * createManipulatorEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState>> manipulatorEventServiceOutEventTestHandler) = 0;
	virtual Smart::IPushServerPattern<CommManipulatorObjects::CommMobileManipulatorState> * createPosePushServer(const std::string &serviceName) = 0;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * createPoseQueryServer(const std::string &serviceName) = 0;
	virtual Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms> * createProgramQuery(const std::string &serviceName) = 0;
	virtual Smart::ISendServerPattern<CommManipulatorObjects::CommManipulatorTrajectory> * createTrajectorySendServer(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTWEBOTSURSERVER_PORTFACTORYINTERFACE_HH_ */
