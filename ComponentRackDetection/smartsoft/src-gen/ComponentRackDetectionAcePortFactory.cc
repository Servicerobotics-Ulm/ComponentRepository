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

#include "ComponentRackDetectionAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentRackDetectionAcePortFactory acePortFactory;

ComponentRackDetectionAcePortFactory::ComponentRackDetectionAcePortFactory()
{  
	componentImpl = 0;
	ComponentRackDetection::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentRackDetectionAcePortFactory::~ComponentRackDetectionAcePortFactory()
{  }

void ComponentRackDetectionAcePortFactory::initialize(ComponentRackDetection *component, int argc, char* argv[])
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
		componentImpl = new ComponentRackDetectionImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentRackDetectionImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentRackDetectionAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> * ComponentRackDetectionAcePortFactory::createKinectQueryClient()
{
	return new SmartACE::QueryClient<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage>(componentImpl);
}


Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> * ComponentRackDetectionAcePortFactory::createEnvironmentQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState> * ComponentRackDetectionAcePortFactory::createObjectEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState>> objectEventServerEventTestHandler)
{
	return new SmartACE::EventServer<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState>(componentImpl, serviceName, objectEventServerEventTestHandler);
}

Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> * ComponentRackDetectionAcePortFactory::createObjectPropertyQueryServer(const std::string &serviceName)
{
	return new SmartACE::QueryServer<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentRackDetectionAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentRackDetectionAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentRackDetectionAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}