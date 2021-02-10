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

#include "SmartGazeboBaseServerOpcUaBackendPortFactory.hh"

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
#include "CommBasicObjectsOpcUa/CommBasePositionUpdateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBaseStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBatteryEventOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBatteryParameterOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBatteryStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBumperEventParameterOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBumperEventResultOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommBumperEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommMobileLaserScanOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommNavigationVelocityOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommVoidOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static SmartGazeboBaseServerOpcUaBackendPortFactory OpcUaBackendPortFactory;

SmartGazeboBaseServerOpcUaBackendPortFactory::SmartGazeboBaseServerOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	SmartGazeboBaseServer::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

SmartGazeboBaseServerOpcUaBackendPortFactory::~SmartGazeboBaseServerOpcUaBackendPortFactory()
{  }

void SmartGazeboBaseServerOpcUaBackendPortFactory::initialize(SmartGazeboBaseServer *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int SmartGazeboBaseServerOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&SmartGazeboBaseServerOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}


Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * SmartGazeboBaseServerOpcUaBackendPortFactory::createBaseSatateQueryAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IPushServerPattern<CommBasicObjects::CommBaseState> * SmartGazeboBaseServerOpcUaBackendPortFactory::createBaseStateServiceOut(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::PushServer<CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> * SmartGazeboBaseServerOpcUaBackendPortFactory::createBatteryEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>> batteryEventServiceOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>(componentImpl, serviceName, batteryEventServiceOutEventTestHandler);
}

Smart::IEventServerPattern<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState> * SmartGazeboBaseServerOpcUaBackendPortFactory::createBumperEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState>> bumperEventServiceOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState>(componentImpl, serviceName, bumperEventServiceOutEventTestHandler);
}

Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> * SmartGazeboBaseServerOpcUaBackendPortFactory::createLaserServiceOut(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::PushServer<CommBasicObjects::CommMobileLaserScan>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommBasicObjects::CommBasePositionUpdate> * SmartGazeboBaseServerOpcUaBackendPortFactory::createLocalizationUpdateServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommBasicObjects::CommBasePositionUpdate>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * SmartGazeboBaseServerOpcUaBackendPortFactory::createNavVelServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommBasicObjects::CommNavigationVelocity>(componentImpl, serviceName);
}


int SmartGazeboBaseServerOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int SmartGazeboBaseServerOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void SmartGazeboBaseServerOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}
