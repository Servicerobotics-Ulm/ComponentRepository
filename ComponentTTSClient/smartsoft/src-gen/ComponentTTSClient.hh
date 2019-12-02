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
#ifndef _COMPONENTTTSCLIENT_HH
#define _COMPONENTTTSCLIENT_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentTTSClientCore.hh"

#include "ComponentTTSClientPortFactoryInterface.hh"
#include "ComponentTTSClientExtension.hh"

// forward declarations
class ComponentTTSClientPortFactoryInterface;
class ComponentTTSClientExtension;

// includes for OpcUaBackendComponentGeneratorExtension

// includes for ComponentTTSClientROSExtension

// includes for PlainOpcUaComponentTTSClientExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommPropertySet.hh>
#include <CommBasicObjects/CommPropertySetACE.hh>
#include <DomainSpeech/CommSpeechOutputMessage.hh>
#include <DomainSpeech/CommSpeechOutputMessageACE.hh>

// include tasks
#include "ConsoleTask.hh"
// include UpcallManagers

// include input-handler(s)
// include request-handler(s)

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentTTSClient::instance()

class ComponentTTSClient : public ComponentTTSClientCore {
private:
	static ComponentTTSClient *_componentTTSClient;
	
	// constructor
	ComponentTTSClient();
	
	// copy-constructor
	ComponentTTSClient(const ComponentTTSClient& cc);
	
	// destructor
	~ComponentTTSClient() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentTTSClientPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentTTSClientExtension*> componentExtensionRegistry;
	
public:
	ParameterStateStruct getGlobalState() const
	{
		return paramHandler.getGlobalState();
	}
	
	ParameterStateStruct getParameters() const
	{
		return paramHandler.getGlobalState();
	}
	
	// define tasks
	Smart::TaskTriggerSubject* consoleTaskTrigger;
	ConsoleTask *consoleTask;
	
	// define input-ports
	
	// define request-ports
	Smart::IQueryClientPattern<DomainSpeech::CommSpeechOutputMessage, CommBasicObjects::CommPropertySet> *speechQueryServiceReq;
	
	// define input-handler
	
	// define output-ports
	Smart::ISendClientPattern<DomainSpeech::CommSpeechOutputMessage> *speechSendServiceOut;
	
	// define answer-ports
	
	// define request-handlers
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of ComponentTTSClientROSExtension
	
	// definitions of PlainOpcUaComponentTTSClientExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentTTSClientPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentTTSClientExtension *extension);
	
	/// this method allows to access the registered component-extensions (automatically converting to the actuall implementation type)
	template <typename T>
	T* getExtension(const std::string &name) {
		auto it = componentExtensionRegistry.find(name);
		if(it != componentExtensionRegistry.end()) {
			return dynamic_cast<T*>(it->second);
		}
		return 0;
	}
	
	/// initialize component's internal members
	void init(int argc, char *argv[]);
	
	/// execute the component's infrastructure
	void run();
	
	/// clean-up component's resources
	void fini();
	
	/// call this method to set the overall component into the Alive state (i.e. component is then ready to operate)
	void setStartupFinished();
	
	/// connect all component's client ports
	Smart::StatusCode connectAndStartAllServices();
	
	/// start all assocuated Activities
	void startAllTasks();
	
	/// start all associated timers
	void startAllTimers();
	
	Smart::StatusCode connectSpeechQueryServiceReq(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectSpeechSendServiceOut(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentTTSClient* instance()
	{
		if(_componentTTSClient == 0) {
			_componentTTSClient = new ComponentTTSClient();
		}
		return _componentTTSClient;
	}
	
	static void deleteInstance() {
		if(_componentTTSClient != 0) {
			delete _componentTTSClient;
		}
	}
	
	// connections parameter
	struct connections_struct
	{
		// component struct
		struct component_struct
		{
			// the name of the component
			std::string name;
			std::string initialComponentMode;
			std::string defaultScheduler;
			bool useLogger;
		} component;
		
		//--- task parameter ---
		struct ConsoleTask_struct {
			double minActFreq;
			double maxActFreq;
			std::string trigger;
			// only one of the following two params is 
			// actually used at run-time according 
			// to the system config model
			double periodicActFreq;
			// or
			std::string inPortRef;
			int prescale;
			// scheduling parameters
			std::string scheduler;
			int priority;
			int cpuAffinity;
		} consoleTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
	
		//--- client port parameter ---
		struct SpeechQueryServiceReq_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} speechQueryServiceReq;
		struct SpeechSendServiceOut_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} speechSendServiceOut;
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for ComponentTTSClientROSExtension
		
		// -- parameters for PlainOpcUaComponentTTSClientExtension
		
	} connections;
};
#endif
