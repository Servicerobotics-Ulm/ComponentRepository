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

#include "ComponentRobotinoImageServerAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentRobotinoImageServerAcePortFactory acePortFactory;

ComponentRobotinoImageServerAcePortFactory::ComponentRobotinoImageServerAcePortFactory()
{  
	componentImpl = 0;
	ComponentRobotinoImageServer::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentRobotinoImageServerAcePortFactory::~ComponentRobotinoImageServerAcePortFactory()
{  }

void ComponentRobotinoImageServerAcePortFactory::initialize(ComponentRobotinoImageServer *component, int argc, char* argv[])
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
		componentImpl = new ComponentRobotinoImageServerImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentRobotinoImageServerImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentRobotinoImageServerAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentRobotinoImageServerAcePortFactory::createBaseStateServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommDevicePoseState> * ComponentRobotinoImageServerAcePortFactory::createDevicePoseUpdateServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommDevicePoseState>(componentImpl);
}


Smart::IPushServerPattern<DomainVision::CommVideoImage> * ComponentRobotinoImageServerAcePortFactory::createRGBImagePushServiceOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<DomainVision::CommVideoImage>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommVideoImage> * ComponentRobotinoImageServerAcePortFactory::createRGBImageQueryServiceAnsw(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, DomainVision::CommVideoImage>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentRobotinoImageServerAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentRobotinoImageServerAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentRobotinoImageServerAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
