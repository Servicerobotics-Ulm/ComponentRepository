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

#include "SmartFestoFleetComOpcUaBackendPortFactory.hh"

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
#include "CommBasicObjectsOpcUa/CommLaserSafetyEventParamOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommLaserSafetyEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommLaserSafetyFieldOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommTaskEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommTaskMessageOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static SmartFestoFleetComOpcUaBackendPortFactory OpcUaBackendPortFactory;

SmartFestoFleetComOpcUaBackendPortFactory::SmartFestoFleetComOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	SmartFestoFleetCom::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

SmartFestoFleetComOpcUaBackendPortFactory::~SmartFestoFleetComOpcUaBackendPortFactory()
{  }

void SmartFestoFleetComOpcUaBackendPortFactory::initialize(SmartFestoFleetCom *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int SmartFestoFleetComOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&SmartFestoFleetComOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}

Smart::IEventClientPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField> * SmartFestoFleetComOpcUaBackendPortFactory::createLaserSafetyEventServiceIn()
{
	return new SeRoNet::OPCUA::Client::EventClient<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField>(componentImpl);
}

Smart::IPushClientPattern<CommBasicObjects::CommTaskMessage> * SmartFestoFleetComOpcUaBackendPortFactory::createTaskResultIn()
{
	return new SeRoNet::OPCUA::Client::PushClient<CommBasicObjects::CommTaskMessage>(componentImpl);
}

Smart::IQueryClientPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> * SmartFestoFleetComOpcUaBackendPortFactory::createKbQueryClient()
{
	return new SeRoNet::OPCUA::Client::QueryClient<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse>(componentImpl);
}


Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> * SmartFestoFleetComOpcUaBackendPortFactory::createTaskEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> taskEventOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>(componentImpl, serviceName, taskEventOutEventTestHandler);
}


int SmartFestoFleetComOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int SmartFestoFleetComOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void SmartFestoFleetComOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}
