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

#include "ComponentTTSOpcUaBackendPortFactory.hh"

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
#include "CommBasicObjectsOpcUa/CommPropertySetOpcUa.hh"
#include "DomainSpeechOpcUa/CommSpeechOutputMessageOpcUa.hh"

// create a static instance of the OpcUaBackendPortFactory
static ComponentTTSOpcUaBackendPortFactory OpcUaBackendPortFactory;

ComponentTTSOpcUaBackendPortFactory::ComponentTTSOpcUaBackendPortFactory()
{  
	componentImpl = 0;
	ComponentTTS::instance()->addPortFactory("OpcUa_SeRoNet", this);
}

ComponentTTSOpcUaBackendPortFactory::~ComponentTTSOpcUaBackendPortFactory()
{  }

void ComponentTTSOpcUaBackendPortFactory::initialize(ComponentTTS *component, int argc, char* argv[])
{
	componentImpl = new SeRoNet::Utils::Component(component->connections.component.name);
}

int ComponentTTSOpcUaBackendPortFactory::onStartup()
{
	if (!component_thread.joinable()) {
    	component_thread = std::thread(&ComponentTTSOpcUaBackendPortFactory::task_execution, this);
    	return 0;
    }
	return -1;
}


Smart::IQueryServerPattern<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet> * ComponentTTSOpcUaBackendPortFactory::createSpeechQueryServiceAnsw(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::QueryServer<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet>(componentImpl, serviceName);
}

Smart::ISendServerPattern<DomainSpeech::CommSpeechOutputMessage> * ComponentTTSOpcUaBackendPortFactory::createSpeechSendServiceIn(const std::string &serviceName)
{
	return new SeRoNet::OPCUA::Server::SendServer<DomainSpeech::CommSpeechOutputMessage>(componentImpl, serviceName);
}


int ComponentTTSOpcUaBackendPortFactory::task_execution()
{
	componentImpl->run();
	return 0;
}

int ComponentTTSOpcUaBackendPortFactory::onShutdown(const std::chrono::steady_clock::duration &timeoutTime)
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

void ComponentTTSOpcUaBackendPortFactory::destroy()
{
	// clean-up component's internally used resources
	delete componentImpl;
}