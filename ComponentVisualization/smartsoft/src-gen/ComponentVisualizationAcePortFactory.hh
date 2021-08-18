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

#ifndef COMPONENTVISUALIZATION_ACE_PORTFACTORY_HH_
#define COMPONENTVISUALIZATION_ACE_PORTFACTORY_HH_

// include ACE/SmartSoft component implementation
#include "ComponentVisualizationImpl.hh"

// include the main component-definition class
#include "ComponentVisualizationPortFactoryInterface.hh"

class ComponentVisualizationAcePortFactory: public ComponentVisualizationPortFactoryInterface
{
private:
	ComponentVisualizationImpl *componentImpl;
public:
	ComponentVisualizationAcePortFactory();
	virtual ~ComponentVisualizationAcePortFactory();

	virtual void initialize(ComponentVisualization *component, int argc, char* argv[]) override;
	virtual int onStartup() override;

	virtual Smart::IPushClientPattern<CommLocalizationObjects::CommAmclVisualizationInfo> * createAmclVisualizationInfoIn() override;
	virtual Smart::IPushClientPattern<CommTrackingObjects::CommDetectedMarkerList> * createMarkerListDetectionServiceIn() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * createRGBDImageQueryServiceReq() override;
	virtual Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommLocalizationObjects::CommVisualLocalizationFeatureMap> * createVisualMarkers() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * createBaseClient() override;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * createCurPushClient() override;
	virtual Smart::IPushClientPattern<DomainVision::CommDepthImage> * createDepthPushNewestClient() override;
	virtual Smart::IPushClientPattern<DomainVision::CommVideoImage> * createImagePushNewestClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileIRScan> * createIrPushNewestClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient1() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient2() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * createLaserClient3() override;
	virtual Smart::IQueryClientPattern<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap> * createLtmQueryClient() override;
	virtual Smart::IEventClientPattern<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonDetectionEventResult> * createPersonDetectionEventClient() override;
	virtual Smart::IQueryClientPattern<CommTrackingObjects::CommPersonId, CommTrackingObjects::CommDetectedPerson> * createPersonDetectionQueryClient() override;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommPlannerGoal> * createPlannerGoalPushClient() override;
	virtual Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * createPlannerWavefrontGridMap() override;
	virtual Smart::IPushClientPattern<DomainVision::CommRGBDImage> * createRgbdPushNewestClient() override;
	virtual Smart::IPushClientPattern<DomainVision::CommDepthImage> * createRgbdQueryClient() override;
	virtual Smart::IPushClientPattern<CommBasicObjects::CommMobileUltrasonicScan> * createUltrasonicPushNewestClient() override;
	
	
	// get a pointer to the internal component implementation
	SmartACE::SmartComponent* getComponentImpl();

	virtual int onShutdown(const std::chrono::steady_clock::duration &timeoutTime=std::chrono::seconds(2)) override;
	virtual void destroy() override;
};

#endif /* COMPONENTVISUALIZATION_ACE_PORTFACTORY_HH_ */
