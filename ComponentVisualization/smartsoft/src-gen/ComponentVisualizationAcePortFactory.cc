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

#include "ComponentVisualizationAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentVisualizationAcePortFactory acePortFactory;

ComponentVisualizationAcePortFactory::ComponentVisualizationAcePortFactory()
{  
	componentImpl = 0;
	ComponentVisualization::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentVisualizationAcePortFactory::~ComponentVisualizationAcePortFactory()
{  }

void ComponentVisualizationAcePortFactory::initialize(ComponentVisualization *component, int argc, char* argv[])
{
	if(component->connections.component.defaultScheduler != "DEFAULT") {
		ACE_Sched_Params sched_params(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(component->connections.component.defaultScheduler == "FIFO") {
			sched_params.policy(ACE_SCHED_FIFO);
			sched_params.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(component->connections.component.defaultScheduler == "RR") {
			sched_params.policy(ACE_SCHED_RR);
			sched_params.priority(ACE_THR_PRI_RR_MIN);
		}
		// create new instance of the SmartSoft component with customized scheuling parameters 
		componentImpl = new ComponentVisualizationImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentVisualizationImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentVisualizationAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommLocalizationObjects::CommAmclVisualizationInfo> * ComponentVisualizationAcePortFactory::createAmclVisualizationInfoIn()
{
	return new SmartACE::PushClient<CommLocalizationObjects::CommAmclVisualizationInfo>(componentImpl);
}

Smart::IPushClientPattern<CommTrackingObjects::CommDetectedMarkerList> * ComponentVisualizationAcePortFactory::createMarkerListDetectionServiceIn()
{
	return new SmartACE::PushClient<CommTrackingObjects::CommDetectedMarkerList>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * ComponentVisualizationAcePortFactory::createRGBDImageQueryServiceReq()
{
	return new SmartACE::QueryClient<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentVisualizationAcePortFactory::createBaseClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * ComponentVisualizationAcePortFactory::createCurPushClient()
{
	return new SmartACE::PushClient<CommNavigationObjects::CommGridMap>(componentImpl);
}

Smart::IPushClientPattern<DomainVision::CommDepthImage> * ComponentVisualizationAcePortFactory::createDepthPushNewestClient()
{
	return new SmartACE::PushClient<DomainVision::CommDepthImage>(componentImpl);
}

Smart::IPushClientPattern<DomainVision::CommVideoImage> * ComponentVisualizationAcePortFactory::createImagePushNewestClient()
{
	return new SmartACE::PushClient<DomainVision::CommVideoImage>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileIRScan> * ComponentVisualizationAcePortFactory::createIrPushNewestClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileIRScan>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * ComponentVisualizationAcePortFactory::createLaserClient1()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * ComponentVisualizationAcePortFactory::createLaserClient2()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * ComponentVisualizationAcePortFactory::createLaserClient3()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>(componentImpl);
}

Smart::IQueryClientPattern<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap> * ComponentVisualizationAcePortFactory::createLtmQueryClient()
{
	return new SmartACE::QueryClient<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap>(componentImpl);
}

Smart::IEventClientPattern<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonDetectionEventResult> * ComponentVisualizationAcePortFactory::createPersonDetectionEventClient()
{
	return new SmartACE::EventClient<CommTrackingObjects::CommPersonLostEventParameter, CommTrackingObjects::CommPersonDetectionEventResult>(componentImpl);
}

Smart::IQueryClientPattern<CommTrackingObjects::CommPersonId, CommTrackingObjects::CommDetectedPerson> * ComponentVisualizationAcePortFactory::createPersonDetectionQueryClient()
{
	return new SmartACE::QueryClient<CommTrackingObjects::CommPersonId, CommTrackingObjects::CommDetectedPerson>(componentImpl);
}

Smart::IPushClientPattern<DomainVision::CommRGBDImage> * ComponentVisualizationAcePortFactory::createRgbdPushNewestClient()
{
	return new SmartACE::PushClient<DomainVision::CommRGBDImage>(componentImpl);
}

Smart::IPushClientPattern<DomainVision::CommDepthImage> * ComponentVisualizationAcePortFactory::createRgbdQueryClient()
{
	return new SmartACE::PushClient<DomainVision::CommDepthImage>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileUltrasonicScan> * ComponentVisualizationAcePortFactory::createUltrasonicPushNewestClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileUltrasonicScan>(componentImpl);
}



SmartACE::SmartComponent* ComponentVisualizationAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentVisualizationAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentVisualizationAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
