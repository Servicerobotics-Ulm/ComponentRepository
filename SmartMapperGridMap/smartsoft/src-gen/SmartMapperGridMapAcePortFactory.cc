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

#include "SmartMapperGridMapAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static SmartMapperGridMapAcePortFactory acePortFactory;

SmartMapperGridMapAcePortFactory::SmartMapperGridMapAcePortFactory()
{  
	componentImpl = 0;
	SmartMapperGridMap::instance()->addPortFactory("ACE_SmartSoft", this);
}

SmartMapperGridMapAcePortFactory::~SmartMapperGridMapAcePortFactory()
{  }

void SmartMapperGridMapAcePortFactory::initialize(SmartMapperGridMap *component, int argc, char* argv[])
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
		componentImpl = new SmartMapperGridMapImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new SmartMapperGridMapImpl(component->connections.component.name, argc, argv);
	}
}

int SmartMapperGridMapAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> * SmartMapperGridMapAcePortFactory::createLaserServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommMobileLaserScan>(componentImpl);
}


Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> * SmartMapperGridMapAcePortFactory::createCurrMapOut(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommNavigationObjects::CommGridMap>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap> * SmartMapperGridMapAcePortFactory::createCurrQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap> * SmartMapperGridMapAcePortFactory::createLtmQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommNavigationObjects::CommGridMapRequest, CommNavigationObjects::CommGridMap>(componentImpl, serviceName);
}


SmartACE::SmartComponent* SmartMapperGridMapAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int SmartMapperGridMapAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void SmartMapperGridMapAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
