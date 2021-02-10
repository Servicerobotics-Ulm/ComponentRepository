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

#include "ComponentWebotsMobileRobotOpcUaBackendPortFactory.hh"

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
#include "CommBasicObjectsOpcUa/CommDigitalInputEventParameterOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommDigitalInputEventResultOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommDigitalInputEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommIOValuesOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommLaserSafetyEventParamOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommLaserSafetyEventStateOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommLaserSafetyFieldOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommNavigationVelocityOpcUa.hh"
#include "CommBasicObjectsOpcUa/CommVoidOpcUa.hh"
#include "CommLocalizationObjectsOpcUa/CommLocalizationEventParameterOpcUa.hh"
#include "CommLocalizationObjectsOpcUa/CommLocalizationEventResultOpcUa.hh"
#include "CommLocalizationObjectsOpcUa/LocalizationEventStateOpcUa.hh"
#include "CommRobotinoObjectsOpcUa/CommRobotinoPowerOutputValueOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static ComponentWebotsMobileRobotOpcUaBackendPortFactory OpcUaBackendPortFactory;

ComponentWebotsMobileRobotOpcUaBackendPortFactory::ComponentWebotsMobileRobotOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	ComponentWebotsMobileRobot::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

ComponentWebotsMobileRobotOpcUaBackendPortFactory::~ComponentWebotsMobileRobotOpcUaBackendPortFactory()
{  }

void ComponentWebotsMobileRobotOpcUaBackendPortFactory::initialize(ComponentWebotsMobileRobot *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int ComponentWebotsMobileRobotOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&ComponentWebotsMobileRobotOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}

Smart::IEventClientPattern<CommLocalizationObjects::CommLocalizationEventParameter, CommLocalizationObjects::CommLocalizationEventResult> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createLocalizationEventServiceIn()
{
	return new SeRoNet::OPCUA::Client::EventClient<CommLocalizationObjects::CommLocalizationEventParameter, CommLocalizationObjects::CommLocalizationEventResult>(componentImpl);
}


Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createBaseStateQueryServiceAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IPushServerPattern<CommBasicObjects::CommBaseState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createBaseStateServiceOut(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::PushServer<CommBasicObjects::CommBaseState>(componentImpl, serviceName);
}

Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createBatteryEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>> batteryEventServiceOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState>(componentImpl, serviceName, batteryEventServiceOutEventTestHandler);
}

Smart::IEventServerPattern<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createBumperEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState>> bumperEventServiceOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommBumperEventParameter, CommBasicObjects::CommBumperEventResult, CommBasicObjects::CommBumperEventState>(componentImpl, serviceName, bumperEventServiceOutEventTestHandler);
}

Smart::IEventServerPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createDigitalInputEventOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>> digitalInputEventOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>(componentImpl, serviceName, digitalInputEventOutEventTestHandler);
}

Smart::IEventServerPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createLaserSafetyEventServiceOut(const std::string &serviceName, std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState>> laserSafetyEventServiceOutEventTestHandler)
{
	return new SeRoNet::OPCUA::Server::EventServer<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState>(componentImpl, serviceName, laserSafetyEventServiceOutEventTestHandler);
}

Smart::ISendServerPattern<CommBasicObjects::CommBasePositionUpdate> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createLocalizationUpdateServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommBasicObjects::CommBasePositionUpdate>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommBasicObjects::CommNavigationVelocity> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createNavigationVelocityServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommBasicObjects::CommNavigationVelocity>(componentImpl, serviceName);
}

Smart::ISendServerPattern<CommRobotinoObjects::CommRobotinoPowerOutputValue> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createPowerOutputSendIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<CommRobotinoObjects::CommRobotinoPowerOutputValue>(componentImpl, serviceName);
}

Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> * ComponentWebotsMobileRobotOpcUaBackendPortFactory::createRobotinoIOValuesQueryServiceAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues>(componentImpl, serviceName);
}


int ComponentWebotsMobileRobotOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int ComponentWebotsMobileRobotOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void ComponentWebotsMobileRobotOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}
