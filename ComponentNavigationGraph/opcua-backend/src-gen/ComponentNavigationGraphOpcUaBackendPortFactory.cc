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

#include "ComponentNavigationGraphOpcUaBackendPortFactory.hh"

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
#include "DomainRobotFleetNavigationOpcUa/CommCorridorPathOpcUa.hh"
#include "DomainRobotFleetNavigationOpcUa/CommNavPathOpcUa.hh"
#include "DomainRobotFleetNavigationOpcUa/CommNodeStatusUpdateAnswerOpcUa.hh"
#include "DomainRobotFleetNavigationOpcUa/CommNodeStatusUpdateRequestOpcUa.hh"
#include "DomainRobotFleetNavigationOpcUa/CommShortestPathRequestOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static ComponentNavigationGraphOpcUaBackendPortFactory OpcUaBackendPortFactory;

ComponentNavigationGraphOpcUaBackendPortFactory::ComponentNavigationGraphOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	ComponentNavigationGraph::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

ComponentNavigationGraphOpcUaBackendPortFactory::~ComponentNavigationGraphOpcUaBackendPortFactory()
{  }

void ComponentNavigationGraphOpcUaBackendPortFactory::initialize(ComponentNavigationGraph *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int ComponentNavigationGraphOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&ComponentNavigationGraphOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}


Smart::ISendServerPattern<DomainRobotFleetNavigation::CommNavPath> * ComponentNavigationGraphOpcUaBackendPortFactory::createNavPathServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<DomainRobotFleetNavigation::CommNavPath>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<DomainRobotFleetNavigation::CommNodeStatusUpdateRequest, DomainRobotFleetNavigation::CommNodeStatusUpdateAnswer> * ComponentNavigationGraphOpcUaBackendPortFactory::createNodeStatusUpdateServiceAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<DomainRobotFleetNavigation::CommNodeStatusUpdateRequest, DomainRobotFleetNavigation::CommNodeStatusUpdateAnswer>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<DomainRobotFleetNavigation::CommShortestPathRequest, DomainRobotFleetNavigation::CommCorridorPath> * ComponentNavigationGraphOpcUaBackendPortFactory::createShortestPathRequestServiceAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<DomainRobotFleetNavigation::CommShortestPathRequest, DomainRobotFleetNavigation::CommCorridorPath>(componentImpl, serviceName);
}


int ComponentNavigationGraphOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int ComponentNavigationGraphOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void ComponentNavigationGraphOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}