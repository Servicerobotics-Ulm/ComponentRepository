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

#include "ComponentWebotsMpsDockingAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentWebotsMpsDockingAcePortFactory acePortFactory;

ComponentWebotsMpsDockingAcePortFactory::ComponentWebotsMpsDockingAcePortFactory()
{  
	componentImpl = 0;
	ComponentWebotsMpsDocking::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentWebotsMpsDockingAcePortFactory::~ComponentWebotsMpsDockingAcePortFactory()
{  }

void ComponentWebotsMpsDockingAcePortFactory::initialize(ComponentWebotsMpsDocking *component, int argc, char* argv[])
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
		componentImpl = new ComponentWebotsMpsDockingImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentWebotsMpsDockingImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentWebotsMpsDockingAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentWebotsMpsDockingAcePortFactory::createBaseStateServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * ComponentWebotsMpsDockingAcePortFactory::createLaserServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>(componentImpl);
}

Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> * ComponentWebotsMpsDockingAcePortFactory::createNavigationVelocityServiceOut()
{
	return new SmartACE::SendClient<CommBasicObjects::CommNavigationVelocity>(componentImpl);
}


Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState> * ComponentWebotsMpsDockingAcePortFactory::createRobotDockingEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState>> robotDockingEventServiceOutEventTestHandler)
{
	return new SmartACE::EventServer<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState>(componentImpl, serviceName, robotDockingEventServiceOutEventTestHandler);
}

Smart::IPushServerPattern<CommBasicObjects::CommTrafficLights> * ComponentWebotsMpsDockingAcePortFactory::createTrafficLightsServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommBasicObjects::CommTrafficLights>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentWebotsMpsDockingAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentWebotsMpsDockingAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentWebotsMpsDockingAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}