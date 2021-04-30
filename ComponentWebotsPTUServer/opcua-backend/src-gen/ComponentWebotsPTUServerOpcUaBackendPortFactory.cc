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

#include "ComponentWebotsPTUServerOpcUaBackendPortFactory.hh"

// include all potentially required pattern implementations
#include <SeRoNetSDK/SeRoNet/OPCUA/Client/PushClient.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Client/EventClient.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Client/QClientOPCUA.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Client/SendClient.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Client/QueryClient.hpp>

#include <SeRoNetSDK/SeRoNet/OPCUA/Server/PushServer.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Server/EventServer.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Server/SendServer.hpp>
#include <SeRoNetSDK/SeRoNet/OPCUA/Server/QueryServer.hpp>

// include referenced CommunicationObject SeRoNetSDK self description implementations
#include "CommBasicObjectsOpcUa/CommBaseStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommDevicePoseStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommVoidOpcUa.hh"
#include "DomainPTUOpcUa/CommMobilePTUStateOpcUa.hh"
#include "DomainPTUOpcUa/CommPTUGoalEventParameterOpcUa.hh"
#include "DomainPTUOpcUa/CommPTUGoalEventResultOpcUa.hh"
#include "DomainPTUOpcUa/CommPTUMoveRequestOpcUa.hh"
#include "DomainPTUOpcUa/CommPTUMoveResponseOpcUa.hh"
#include "DomainPTUOpcUa/PTUGoalEventStateOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static ComponentWebotsPTUServerOpcUaBackendPortFactory OpcUaBackendPortFactory;

ComponentWebotsPTUServerOpcUaBackendPortFactory::ComponentWebotsPTUServerOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	ComponentWebotsPTUServer::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

ComponentWebotsPTUServerOpcUaBackendPortFactory::~ComponentWebotsPTUServerOpcUaBackendPortFactory()
{  }

void ComponentWebotsPTUServerOpcUaBackendPortFactory::initialize(ComponentWebotsPTUServer *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int ComponentWebotsPTUServerOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&ComponentWebotsPTUServerOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}

Smart::IPushClientPattern<CommBasicObjects::CommBaseState> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createBaseStateClient()
{
	return new SeRoNet::OPCUA::Client::PushClient<CommBasicObjects::CommBaseState>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createBaseStateQueryClient()
{
	return new SeRoNet::OPCUA::Client::QueryClient<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState>(componentImpl);
}


Smart::IPushServerPattern<CommBasicObjects::CommDevicePoseState> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createDevicePoseStateServer(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::PushServer<CommBasicObjects::CommDevicePoseState>(componentImpl, serviceName);
}

Smart::IEventServerPattern<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createGoalEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState>> goalEventServerEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState>(componentImpl, serviceName, goalEventServerEventTestHandler);
}

Smart::IQueryServerPattern<DomainPTU::CommPTUMoveRequest, DomainPTU::CommPTUMoveResponse> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createMoveQueryServer(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<DomainPTU::CommPTUMoveRequest, DomainPTU::CommPTUMoveResponse>(componentImpl, serviceName);
}

Smart::ISendServerPattern<DomainPTU::CommPTUMoveRequest> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createMoveSendServer(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<DomainPTU::CommPTUMoveRequest>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainPTU::CommMobilePTUState> * ComponentWebotsPTUServerOpcUaBackendPortFactory::createStateQueryServer(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<CommBasicObjects::CommVoid, DomainPTU::CommMobilePTUState>(componentImpl, serviceName);
}


int ComponentWebotsPTUServerOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int ComponentWebotsPTUServerOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
{
	// stop component-internal infrastructure
	componentImpl->stopRunning();
	// wait on component thread to exit
	if (component_thread.joinable()) {
		// FIXME: don't wait infinetly (use timeoutTime here)
    	component_thread.join();
    }
	return 0;
}

void ComponentWebotsPTUServerOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}