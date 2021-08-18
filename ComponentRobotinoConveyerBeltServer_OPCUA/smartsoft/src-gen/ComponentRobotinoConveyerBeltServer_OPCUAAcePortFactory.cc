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

#include "ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory acePortFactory;

ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory()
{  
	componentImpl = 0;
	ComponentRobotinoConveyerBeltServer_OPCUA::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::~ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory()
{  }

void ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::initialize(ComponentRobotinoConveyerBeltServer_OPCUA *component, int argc, char* argv[])
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
		componentImpl = new ComponentRobotinoConveyerBeltServer_OPCUAImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentRobotinoConveyerBeltServer_OPCUAImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IEventClientPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult> * ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::createCommDigitalInputEventIn()
{
	return new SmartACE::EventClient<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::createCommIOValuesQueryServiceReq()
{
	return new SmartACE::QueryClient<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>(componentImpl);
}

Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::createCommPowerOutputSendOut()
{
	return new SmartACE::SendClient<CommRobotinoObjects::CommRobotinoPowerOutputValue>(componentImpl);
}


Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::createRobotinoConveyerBeltEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> robotinoConveyerBeltEventOutEventTestHandler)
{
	return new SmartACE::EventServer<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>(componentImpl, serviceName, robotinoConveyerBeltEventOutEventTestHandler);
}


SmartACE::SmartComponent* ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentRobotinoConveyerBeltServer_OPCUAAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
