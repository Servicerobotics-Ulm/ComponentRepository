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

#include "ComponentWebotsURServerAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentWebotsURServerAcePortFactory acePortFactory;

ComponentWebotsURServerAcePortFactory::ComponentWebotsURServerAcePortFactory()
{  
	componentImpl = 0;
	ComponentWebotsURServer::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentWebotsURServerAcePortFactory::~ComponentWebotsURServerAcePortFactory()
{  }

void ComponentWebotsURServerAcePortFactory::initialize(ComponentWebotsURServer *component, int argc, char* argv[])
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
		componentImpl = new ComponentWebotsURServerImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentWebotsURServerImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentWebotsURServerAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentWebotsURServerAcePortFactory::createBaseStateServiceIn()
{
	return new SmartACE::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}


Smart::IEventServerPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState> * ComponentWebotsURServerAcePortFactory::createIoEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>> ioEventServerEventTestHandler)
{
	return new SmartACE::EventServer<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>(componentImpl, serviceName, ioEventServerEventTestHandler);
}

Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * ComponentWebotsURServerAcePortFactory::createIoQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState> * ComponentWebotsURServerAcePortFactory::createManipulatorEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState>> manipulatorEventServiceOutEventTestHandler)
{
	return new SmartACE::EventServer<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState>(componentImpl, serviceName, manipulatorEventServiceOutEventTestHandler);
}

Smart::IPushServerPattern<CommManipulatorObjects::CommMobileManipulatorState> * ComponentWebotsURServerAcePortFactory::createPosePushServer(const std::string &serviceName)
{
	return new SmartACE::PushServer<CommManipulatorObjects::CommMobileManipulatorState>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> * ComponentWebotsURServerAcePortFactory::createPoseQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms> * ComponentWebotsURServerAcePortFactory::createProgramQuery(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommManipulatorObjects::CommManipulatorTrajectory> * ComponentWebotsURServerAcePortFactory::createTrajectorySendServer(const std::string &serviceName)
{
	return new SmartACE::SendServer<CommManipulatorObjects::CommManipulatorTrajectory>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentWebotsURServerAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentWebotsURServerAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentWebotsURServerAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
