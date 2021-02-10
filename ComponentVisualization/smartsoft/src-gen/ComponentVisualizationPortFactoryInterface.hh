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

#ifndef COMPONENTVISUALIZATION_PORTFACTORYINTERFACE_HH_
#define COMPONENTVISUALIZATION_PORTFACTORYINTERFACE_HH_

// include communication objects
#include <CommLocalizationObjects/CommAmclVisualizationInfo.hh>
#include <CommLocalizationObjects/CommAmclVisualizationInfoACE.hh>
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <DomainVision/CommDepthImage.hh>
#include <DomainVision/CommDepthImageACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerList.hh>
#include <CommTrackingObjects/CommDetectedMarkerListACE.hh>
#include <CommTrackingObjects/CommDetectedPerson.hh>
#include <CommTrackingObjects/CommDetectedPersonACE.hh>
#include <CommNavigationObjects/CommGridMap.hh>
#include <CommNavigationObjects/CommGridMapACE.hh>
#include <CommNavigationObjects/CommGridMapRequest.hh>
#include <CommNavigationObjects/CommGridMapRequestACE.hh>
#include <CommBasicObjects/CommMobileIRScan.hh>
#include <CommBasicObjects/CommMobileIRScanACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommMobileUltrasonicScan.hh>
#include <CommBasicObjects/CommMobileUltrasonicScanACE.hh>
#include <CommTrackingObjects/CommPersonDetectionEventResult.hh>
#include <CommTrackingObjects/CommPersonDetectionEventResultACE.hh>
#include <CommTrackingObjects/CommPersonId.hh>
#include <CommTrackingObjects/CommPersonIdACE.hh>
#include <CommTrackingObjects/CommPersonLostEventParameter.hh>
#include <CommTrackingObjects/CommPersonLostEventParameterACE.hh>
#include <DomainVision/CommRGBDImage.hh>
#include <DomainVision/CommRGBDImageACE.hh>
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>
#include <CommTrackingObjects/PersonLostEventState.hh>
#include <CommTrackingObjects/PersonLostEventStateACE.hh>

#include <chrono>

// include component's main class
#include "ComponentVisualization.hh"

// forward declaration
class ComponentVisualization;

class ComponentVisualizationPortFactoryInterface {
public:
	ComponentVisualizationPortFactoryInterface() { };
	virtual ~ComponentVisualizationPortFactoryInterface() { };

	virtual void initialize(ComponentVisualization *component, int argc, char* argv[]) = 0;
	virtual int onStartup() = 0;

	virtual Smart::IPushClientPattern<CommLocalizationObjects::CommAmclVisualizationInfo> * createAmclVisualizationInfoIn() = 0;
	virtual Smart::IPushClientPattern<CommTrackingObjects::CommDetectedMarkerList> * createMarkerListDetectionServiceIn() = 0;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * createRGBDImageQueryServiceReq() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseClient() = 0;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * createCurPushClient() = 0;
	virtual Smart::IPushClientPattern<DomainVision::CommDepthImage> * createDepthPushNewestClient() = 0;
	virtual Smart::IPushClientPattern<DomainVision::CommVideoImage> * createImagePushNewestClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileIRScan> * createIrPushNewestClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient1() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient2() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient3() = 0;
	virtual Smart::IQueryClientPattern<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap> * createLtmQueryClient() = 0;
	virtual Smart::IEventClientPattern<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonDetectionEventResult> * createPersonDetectionEventClient() = 0;
	virtual Smart::IQueryClientPattern<CommTrackingObjects::CommPersonId, CommTrackingObjects::CommDetectedPerson> * createPersonDetectionQueryClient() = 0;
	virtual Smart::IPushClientPattern<DomainVision::CommRGBDImage> * createRgbdPushNewestClient() = 0;
	virtual Smart::IPushClientPattern<DomainVision::CommDepthImage> * createRgbdQueryClient() = 0;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileUltrasonicScan> * createUltrasonicPushNewestClient() = 0;
	

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) = 0;
	virtual void destroy() = 0;
};

#endif /* COMPONENTVISUALIZATION_PORTFACTORYINTERFACE_HH_ */
