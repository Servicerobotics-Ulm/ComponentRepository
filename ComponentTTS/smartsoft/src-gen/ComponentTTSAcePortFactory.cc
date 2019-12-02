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

#include "ComponentTTSAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentTTSAcePortFactory acePortFactory;

ComponentTTSAcePortFactory::ComponentTTSAcePortFactory()
{  
	componentImpl = 0;
	ComponentTTS::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentTTSAcePortFactory::~ComponentTTSAcePortFactory()
{  }

void ComponentTTSAcePortFactory::initialize(ComponentTTS *component, int argc, char* argv[])
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
		componentImpl = new ComponentTTSImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentTTSImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentTTSAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}


Smart::IQueryServerPattern<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet> * ComponentTTSAcePortFactory::createSpeechQueryServiceAnsw(const std::string &serviceName)
{
	return new SmartACE::QueryServer<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet>(componentImpl, serviceName);
}

Smart::ISendServerPattern<DomainSpeech::CommSpeechOutputMessage> * ComponentTTSAcePortFactory::createSpeechSendServiceIn(const std::string &serviceName)
{
	return new SmartACE::SendServer<DomainSpeech::CommSpeechOutputMessage>(componentImpl, serviceName);
}


SmartACE::SmartComponent* ComponentTTSAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentTTSAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentTTSAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}
