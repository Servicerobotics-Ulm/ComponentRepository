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

#include "SmartJobDispatcherOpcUaBackendPortFactory.hh"

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
#include "CommBasicObjectsOpcUa/CommKBRequestOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommKBResponseOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommTaskEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommTaskMessageOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommVoidOpcUa.hh"
#include "CommNavigationObjectsOpcUa/CommNavigationTopologyMapOpcUa.hh"
#include "DomainSymbolicPlannerOpcUa/CommSymbolicPlannerPlanOpcUa.hh"
#include "DomainSymbolicPlannerOpcUa/CommSymbolicPlannerRequestOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static SmartJobDispatcherOpcUaBackendPortFactory OpcUaBackendPortFactory;

SmartJobDispatcherOpcUaBackendPortFactory::SmartJobDispatcherOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	SmartJobDispatcher::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

SmartJobDispatcherOpcUaBackendPortFactory::~SmartJobDispatcherOpcUaBackendPortFactory()
{  }

void SmartJobDispatcherOpcUaBackendPortFactory::initialize(SmartJobDispatcher *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int SmartJobDispatcherOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&SmartJobDispatcherOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}

Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * SmartJobDispatcherOpcUaBackendPortFactory::createCommKBQueryReq()
{
	return new SeRoNet::OPCUA::Client::QueryClient<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommNavigationObjects::CommNavigationTopologyMap> * SmartJobDispatcherOpcUaBackendPortFactory::createNavigationTopologyMapQueryReq()
{
	return new SeRoNet::OPCUA::Client::QueryClient<CommBasicObjects::CommVoid, CommNavigationObjects::CommNavigationTopologyMap>(componentImpl);
}

Smart::IQueryClientPattern<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan> * SmartJobDispatcherOpcUaBackendPortFactory::createSymbolicPannerServiceReq()
{
	return new SeRoNet::OPCUA::Client::QueryClient<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan>(componentImpl);
}

Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * SmartJobDispatcherOpcUaBackendPortFactory::createJobSource2EventClient()
{
	return new SeRoNet::OPCUA::Client::EventClient<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage>(componentImpl);
}

Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * SmartJobDispatcherOpcUaBackendPortFactory::createJobSource3EventClient()
{
	return new SeRoNet::OPCUA::Client::EventClient<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage>(componentImpl);
}

Smart::IEventClientPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage> * SmartJobDispatcherOpcUaBackendPortFactory::createJobSourceEventClient()
{
	return new SeRoNet::OPCUA::Client::EventClient<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage>(componentImpl);
}


Smart::IPushServerPattern<CommBasicObjects::CommTaskMessage> * SmartJobDispatcherOpcUaBackendPortFactory::createTaskResultOut(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::PushServer<CommBasicObjects::CommTaskMessage>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> * SmartJobDispatcherOpcUaBackendPortFactory::createJobEventServer(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> jobEventServerEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>(componentImpl, serviceName, jobEventServerEventTestHandler);
}

Smart::ISendServerPattern<CommBasicObjects::CommTaskMessage> * SmartJobDispatcherOpcUaBackendPortFactory::createJobResultSendServer(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommBasicObjects::CommTaskMessage>(componentImpl, serviceName);
}


int SmartJobDispatcherOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int SmartJobDispatcherOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void SmartJobDispatcherOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}
