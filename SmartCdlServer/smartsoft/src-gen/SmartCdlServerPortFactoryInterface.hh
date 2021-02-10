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

#ifndef SMARTCDLSERVER_PORTFACTORYINTERFACE_HH_
#define SMARTCDLSERVER_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommNavigationObjects/CdlGoalEventState.hh>
#include <CommNavigationObjects/CdlGoalEventStateACE.hh>
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommNavigationObjects/CommCdlGoalEventParameter.hh>
#include <CommNavigationObjects/CommCdlGoalEventParameterACE.hh>
#include <CommNavigationObjects/CommCdlGoalEventResult.hh>
#include <CommNavigationObjects/CommCdlGoalEventResultACE.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventParameter.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventParameterACE.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventResult.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventResultACE.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedState.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedStateACE.hh>
#include <CommNavigationObjects/CommCorridorNavigationGoal.hh>
#include <CommNavigationObjects/CommCorridorNavigationGoalACE.hh>
#include <CommBasicObjects/CommMobileIRScan.hh>
#include <CommBasicObjects/CommMobileIRScanACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommNavigationVelocityACE.hh>
#include <CommNavigationObjects/CommPlannerGoal.hh>
#include <CommNavigationObjects/CommPlannerGoalACE.hh>
#include <CommTrackingObjects/CommTrackingGoal.hh>
#include <CommTrackingObjects/CommTrackingGoalACE.hh>

#include <chrono>

// include component's main class
#include "SmartCdlServer.hh"

// forward declaration
class SmartCdlServer;

class SmartCdlServerPortFactoryInterface {
public:
	SmartCdlServerPortFactoryInterface() { };
	virtual ~SmartCdlServerPortFactoryInterface() { };

	virtual void initialize(SmartCdlServer *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseStateClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileIRScan> * createIRClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient2() = 0;
	virtual Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> * createNavVelSendClient() = 0;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommCorridorNavigationGoal> * createPathNavigationGoalClient() = 0;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommPlannerGoal> * createPlannerClient() = 0;
	virtual Smart::IPushClientPattern<CommTrackingObjects::CommTrackingGoal> * createTrackingClient() = 0;
	
	virtual Smart::IEventServerPattern<CommNavigationObjects::CommCdlGoalEventParameter, CommNavigationObjects::CommCdlGoalEventResult, CommNavigationObjects::CdlGoalEventState> * createGoalEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommNavigationObjects::CommCdlGoalEventParameter, CommNavigationObjects::CommCdlGoalEventResult, CommNavigationObjects::CdlGoalEventState>> goalEventServerEventTestHandler) = 0;
	virtual Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * createNavVelSendServer(const std::string &serviceName) = 0;
	virtual Smart::IEventServerPattern<CommNavigationObjects::CommCdlRobotBlockedEventParameter, CommNavigationObjects::CommCdlRobotBlockedEventResult, CommNavigationObjects::CommCdlRobotBlockedState> * createRobotBlockedEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommNavigationObjects::CommCdlRobotBlockedEventParameter, CommNavigationObjects::CommCdlRobotBlockedEventResult, CommNavigationObjects::CommCdlRobotBlockedState>> robotBlockedEventServerEventTestHandler) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* SMARTCDLSERVER_PORTFACTORYINTERFACE_HH_ */
