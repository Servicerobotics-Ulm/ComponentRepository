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

#include "SmartFestoFleetComAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static SmartFestoFleetComAcePortFactory acePortFactory;

SmartFestoFleetComAcePortFactory::SmartFestoFleetComAcePortFactory()
{  
	componentImpl = 0;
	SmartFestoFleetCom::instance()->addPortFactory("ACE_SmartSoft", this);
}

SmartFestoFleetComAcePortFactory::~SmartFestoFleetComAcePortFactory()
{  }

void SmartFestoFleetComAcePortFactory::initialize(SmartFestoFleetCom *component, int argc, char* argv[])
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
		componentImpl = new SmartFestoFleetComImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new SmartFestoFleetComImpl(component->connections.component.name, argc, argv);
	}
}

int SmartFestoFleetComAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IEventClientPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField> * SmartFestoFleetComAcePortFactory::createLaserSafetyEventServiceIn()
{
	return new SmartACE::EventClient<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommTaskMessage> * SmartFestoFleetComAcePortFactory::createTaskResultIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommTaskMessage>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * SmartFestoFleetComAcePortFactory::createKbQueryClient()
{
	return new SmartACE::QueryClient<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse>(componentImpl);
}


Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> * SmartFestoFleetComAcePortFactory::createTaskEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> taskEventOutEventTestHandler)
{
	return new SmartACE::EventServer<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>(componentImpl, serviceName, taskEventOutEventTestHandler);
}


SmartACE::SmartComponent* SmartFestoFleetComAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int SmartFestoFleetComAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void SmartFestoFleetComAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}