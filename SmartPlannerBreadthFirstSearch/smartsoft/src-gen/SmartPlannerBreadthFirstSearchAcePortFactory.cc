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

#include "SmartPlannerBreadthFirstSearchAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static SmartPlannerBreadthFirstSearchAcePortFactory acePortFactory;

SmartPlannerBreadthFirstSearchAcePortFactory::SmartPlannerBreadthFirstSearchAcePortFactory()
{  
	componentImpl = 0;
	SmartPlannerBreadthFirstSearch::instance()->addPortFactory("ACE_SmartSoft", this);
}

SmartPlannerBreadthFirstSearchAcePortFactory::~SmartPlannerBreadthFirstSearchAcePortFactory()
{  }

void SmartPlannerBreadthFirstSearchAcePortFactory::initialize(SmartPlannerBreadthFirstSearch *component, int argc, char* argv[])
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
		componentImpl = new SmartPlannerBreadthFirstSearchImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new SmartPlannerBreadthFirstSearchImpl(component->connections.component.name, argc, argv);
	}
}

int SmartPlannerBreadthFirstSearchAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * SmartPlannerBreadthFirstSearchAcePortFactory::createBaseStateClient()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IPushClientPattern<CommNavigationObjects::CommGridMap> * SmartPlannerBreadthFirstSearchAcePortFactory::createCurMapClient()
{
	return new SmartACE::PushClient<CommNavigationObjects::CommGridMap>(componentImpl);
}


Smart::IEventServerPattern<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState> * SmartPlannerBreadthFirstSearchAcePortFactory::createPlannerEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState>> plannerEventServerEventTestHandler)
{
	return new SmartACE::EventServer<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState>(componentImpl, serviceName, plannerEventServerEventTestHandler);
}

Smart::IPushServerPattern<CommNavigationObjects::CommPlannerGoal> * SmartPlannerBreadthFirstSearchAcePortFactory::createPlannerGoalServer(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommNavigationObjects::CommPlannerGoal>(componentImpl, serviceName);
}


SmartACE::SmartComponent* SmartPlannerBreadthFirstSearchAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int SmartPlannerBreadthFirstSearchAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void SmartPlannerBreadthFirstSearchAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
