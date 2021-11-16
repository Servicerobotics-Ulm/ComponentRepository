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

#include "ComponentWebotsOpcUaBackendPortFactory.hh"

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
#include "DomainRobotFleetNavigationOpcUa/CommNavPathOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static ComponentWebotsOpcUaBackendPortFactory OpcUaBackendPortFactory;

ComponentWebotsOpcUaBackendPortFactory::ComponentWebotsOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	ComponentWebots::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

ComponentWebotsOpcUaBackendPortFactory::~ComponentWebotsOpcUaBackendPortFactory()
{  }

void ComponentWebotsOpcUaBackendPortFactory::initialize(ComponentWebots *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int ComponentWebotsOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&ComponentWebotsOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}

Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * ComponentWebotsOpcUaBackendPortFactory::createCommKBQueryReq()
{
	return new SeRoNet::OPCUA::Client::QueryClient<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse>(componentImpl);
}

Smart::ISendClientPattern<DomainRobotFleetNavigation::CommNavPath> * ComponentWebotsOpcUaBackendPortFactory::createNavPathServiceOut()
{
	return new SeRoNet::OPCUA::Client::SendClient<DomainRobotFleetNavigation::CommNavPath>(componentImpl);
}



int ComponentWebotsOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int ComponentWebotsOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void ComponentWebotsOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}
