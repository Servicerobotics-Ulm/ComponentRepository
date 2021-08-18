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

#include "SmartPioneerBaseServerAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static SmartPioneerBaseServerAcePortFactory acePortFactory;

SmartPioneerBaseServerAcePortFactory::SmartPioneerBaseServerAcePortFactory()
{  
	componentImpl = 0;
	SmartPioneerBaseServer::instance()->addPortFactory("ACE_SmartSoft", this);
}

SmartPioneerBaseServerAcePortFactory::~SmartPioneerBaseServerAcePortFactory()
{  }

void SmartPioneerBaseServerAcePortFactory::initialize(SmartPioneerBaseServer *component, int argc, char* argv[])
{
	if(component->connections.component.defaultScheduler != "DEFAULT") {
		ACE_Sched_Params sched_params(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(component->connections.component.defaultScheduler == "FIFO") {
			sched_params.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				sched_params.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				sched_params.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(component->connections.component.defaultScheduler == "RR") {
			sched_params.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				sched_params.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				sched_params.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		// create new instance of the SmartSoft component with customized scheuling parameters 
		componentImpl = new SmartPioneerBaseServerImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new SmartPioneerBaseServerImpl(component->connections.component.name, argc, argv);
	}
}

int SmartPioneerBaseServerAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}


Smart::IPushServerPattern<CommBasicObjects::CommBaseState> * SmartPioneerBaseServerAcePortFactory::createBasePositionOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * SmartPioneerBaseServerAcePortFactory::createBaseStateQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> * SmartPioneerBaseServerAcePortFactory::createBatteryEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>> batteryEventServerEventTestHandler)
{
	return new SmartACE::EventServer<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>(componentImpl, serviceName, batteryEventServerEventTestHandler);
}

Smart::ISendServerPattern<CommBasicObjects::CommBasePositionUpdate> * SmartPioneerBaseServerAcePortFactory::createLocalizationUpdate(const std::string &serviceName)
{
	return new SmartACE::SendServer<CommBasicObjects::CommBasePositionUpdate>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * SmartPioneerBaseServerAcePortFactory::createNavVelIn(const std::string &serviceName)
{
	return new SmartACE::SendServer<CommBasicObjects::CommNavigationVelocity>(componentImpl, serviceName);
}


SmartACE::SmartComponent* SmartPioneerBaseServerAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int SmartPioneerBaseServerAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void SmartPioneerBaseServerAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
