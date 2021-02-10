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

#include "ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory.hh"

// create a static instance of the default AcePortFactory
static ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory acePortFactory;

ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory()
{  
	componentImpl = 0;
	ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN::instance()->addPortFactory("ACE_SmartSoft", this);
}

ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::~ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory()
{  }

void ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::initialize(ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PN *component, int argc, char* argv[])
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
		componentImpl = new ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNImpl(component->connections.component.name, argc, argv, sched_params);
	} else {
		// create new instance of the SmartSoft component
		componentImpl = new ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNImpl(component->connections.component.name, argc, argv);
	}
}

int ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::onStartup()
{
	return componentImpl->startComponentInfrastructure();
}

Smart::IEventClientPattern<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult> * ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::createDigitalInputEventClient()
{
	return new SmartACE::EventClient<CommRobotinoObjects::CommDigitalInputEventParameter, CommRobotinoObjects::CommDigitalInputEventResult>(componentImpl);
}

Smart::IQueryClientPattern<CommRobotinoObjects::CommRobotinoIOValues, CommRobotinoObjects::CommRobotinoIOValues> * ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::createIOQueryClient()
{
	return new SmartACE::QueryClient<CommRobotinoObjects::CommRobotinoIOValues, CommRobotinoObjects::CommRobotinoIOValues>(componentImpl);
}

Smart::ISendClientPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::createPowerOutPutSendClient()
{
	return new SmartACE::SendClient<CommRobotinoObjects::CommRobotinoPowerOutputValue>(componentImpl);
}


Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState> * ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::createLoadEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>> loadEventServerEventTestHandler)
{
	return new SmartACE::EventServer<CommRobotinoObjects::CommRobotinoConveyerBeltEventParameter, CommRobotinoObjects::CommRobotinoConveyerBeltEventResult, CommRobotinoObjects::RobotinoConveyerBeltEventState>(componentImpl, serviceName, loadEventServerEventTestHandler);
}


SmartACE::SmartComponent* ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::getComponentImpl()
{
	return componentImpl;
}

int ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	componentImpl->stopComponentInfrastructure(timeoutTime);
	return 0;
}

void ComponentRobotinoConveyerBeltServerOpcuaCPXECECC1PNAcePortFactory::destroy()
{
	// clean-up component's internally used resources (internally used communication middleware) 
	componentImpl->cleanUpComponentResources();
	delete componentImpl;
}