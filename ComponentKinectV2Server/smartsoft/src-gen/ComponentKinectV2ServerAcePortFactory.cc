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

#include "ComponentKinectV2ServerAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentKinectV2ServerAcePortFactory acePortFactory;

ComponentKinectV2ServerAcePortFactory::ComponentKinectV2ServerAcePortFactory()
{  
	componentImpl = 0;
	ComponentKinectV2Server::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentKinectV2ServerAcePortFactory::~ComponentKinectV2ServerAcePortFactory()
{  }

void ComponentKinectV2ServerAcePortFactory::initialize(ComponentKinectV2Server *component, int argc, char* argv[])
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
		componentImpl = new ComponentKinectV2ServerImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentKinectV2ServerImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentKinectV2ServerAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentKinectV2ServerAcePortFactory::createBasePushTimedClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommDevicePoseState> * ComponentKinectV2ServerAcePortFactory::createPtuPosePushNewestClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommDevicePoseState>(componentImpl);
}


Smart::IPushServerPattern<DomainVision::CommRGBDImage> * ComponentKinectV2ServerAcePortFactory::createRGBDImageQueryServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainVision::CommRGBDImage>(componentImpl, serviceName);
}

Smart::IPushServerPattern<DomainVision::CommVideoImage> * ComponentKinectV2ServerAcePortFactory::createRGBImagePushServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainVision::CommVideoImage>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommVideoImage> * ComponentKinectV2ServerAcePortFactory::createColorImageQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, DomainVision::CommVideoImage>(componentImpl, serviceName);
}

Smart::IPushServerPattern<DomainVision::CommDepthImage> * ComponentKinectV2ServerAcePortFactory::createDepthImagePushServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainVision::CommDepthImage>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * ComponentKinectV2ServerAcePortFactory::createImageQueryV2Server(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentKinectV2ServerAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentKinectV2ServerAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentKinectV2ServerAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
