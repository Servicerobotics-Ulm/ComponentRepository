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

#include "ComponentAprilTagDetectorAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentAprilTagDetectorAcePortFactory acePortFactory;

ComponentAprilTagDetectorAcePortFactory::ComponentAprilTagDetectorAcePortFactory()
{  
	componentImpl = 0;
	ComponentAprilTagDetector::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentAprilTagDetectorAcePortFactory::~ComponentAprilTagDetectorAcePortFactory()
{  }

void ComponentAprilTagDetectorAcePortFactory::initialize(ComponentAprilTagDetector *component, int argc, char* argv[])
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
		componentImpl = new ComponentAprilTagDetectorImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentAprilTagDetectorImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentAprilTagDetectorAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<DomainVision::CommVideoImage> * ComponentAprilTagDetectorAcePortFactory::createRGBImagePushServiceIn()
{
	return new SmartACE::PushClient<DomainVision::CommVideoImage>(componentImpl);
}


Smart::IPushServerPattern<CommTrackingObjects::CommDetectedMarkerList> * ComponentAprilTagDetectorAcePortFactory::createMarkerListDetectionServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommTrackingObjects::CommDetectedMarkerList>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState> * ComponentAprilTagDetectorAcePortFactory::createMarkerListEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState>> markerListEventServiceOutEventTestHandler)
{
	return new SmartACE::EventServer<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState>(componentImpl, serviceName, markerListEventServiceOutEventTestHandler);
}

Smart::IPushServerPattern<DomainVision::CommVideoImage> * ComponentAprilTagDetectorAcePortFactory::createRGBImagePushServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainVision::CommVideoImage>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentAprilTagDetectorAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentAprilTagDetectorAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentAprilTagDetectorAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
