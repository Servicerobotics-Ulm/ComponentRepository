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

#ifndef COMPONENTREALSENSEPERSONTRACKER_PORTFACTORYINTERFACE_HH_
#define COMPONENTREALSENSEPERSONTRACKER_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommTrackingObjects/CommPersonLostEventParameter.hh>
#include <CommTrackingObjects/CommPersonLostEventParameterACE.hh>
#include <CommTrackingObjects/CommPersonLostEventResult.hh>
#include <CommTrackingObjects/CommPersonLostEventResultACE.hh>
#include <DomainVision/CommRGBDImage.hh>
#include <DomainVision/CommRGBDImageACE.hh>
#include <CommTrackingObjects/CommTrackingGoal.hh>
#include <CommTrackingObjects/CommTrackingGoalACE.hh>
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>
#include <CommTrackingObjects/PersonLostEventState.hh>
#include <CommTrackingObjects/PersonLostEventStateACE.hh>

#include <chrono>

// include component's main class
#include "ComponentRealSensePersonTracker.hh"

// forward declaration
class ComponentRealSensePersonTracker;

class ComponentRealSensePersonTrackerPortFactoryInterface {
public:
	ComponentRealSensePersonTrackerPortFactoryInterface() { };
	virtual ~ComponentRealSensePersonTrackerPortFactoryInterface() { };

	virtual void initialize(ComponentRealSensePersonTracker *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<DomainVision::CommRGBDImage> * createRealSenseClient() = 0;
	
	virtual Smart::IEventServerPattern<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonLostEventResult, CommTrackingObjects::PersonLostEventState> * createPersonLostEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonLostEventResult, CommTrackingObjects::PersonLostEventState>> personLostEventServerEventTestHandler) = 0;
	virtual Smart::IPushServerPattern<DomainVision::CommVideoImage> * createRgb_video_server(const std::string &serviceName) = 0;
	virtual Smart::IPushServerPattern<CommTrackingObjects::CommTrackingGoal> * createTrackingGoalServer(const std::string &serviceName) = 0;

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTREALSENSEPERSONTRACKER_PORTFACTORYINTERFACE_HH_ */