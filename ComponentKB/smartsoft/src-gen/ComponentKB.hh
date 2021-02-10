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
#ifndef _COMPONENTKB_HH
#define _COMPONENTKB_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentKBCore.hh"

#include "ComponentKBPortFactoryInterface.hh"
#include "ComponentKBExtension.hh"

// forward declarations
class ComponentKBPortFactoryInterface;
class ComponentKBExtension;

// includes for ComponentKBROS1InterfacesExtension

// includes for ComponentKBROSExtension

// includes for ComponentKBRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentKBExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommKBEventParam.hh>
#include <CommBasicObjects/CommKBEventParamACE.hh>
#include <CommBasicObjects/CommKBEventResult.hh>
#include <CommBasicObjects/CommKBEventResultACE.hh>
#include <CommBasicObjects/CommKBRequest.hh>
#include <CommBasicObjects/CommKBRequestACE.hh>
#include <CommBasicObjects/CommKBResponse.hh>
#include <CommBasicObjects/CommKBResponseACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "Dummy.hh"
// include UpcallManagers and InputCollectors
#include "KbChainedEntriesEventClientUpcallManager.hh"
#include "KbChainedEntriesEventClientInputCollector.hh"

// include input-handler(s)
#include "KbChainedEntriesEventClientHandler.hh"
// include request-handler(s)
#include "KbQueryHandler.hh"
// output port wrappers
#include "KbEventServerWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentKB::instance()

class ComponentKB : public ComponentKBCore {
private:
	static ComponentKB *_componentKB;
	
	// constructor
	ComponentKB();
	
	// copy-constructor
	ComponentKB(const ComponentKB& cc);
	
	// destructor
	~ComponentKB() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentKBPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentKBExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* dummyTrigger;
	Dummy *dummy;
	
	// define input-ports
	// InputPort kbChainedEntriesEventClient
	Smart::IEventClientPattern<CommBasicObjects::CommKBEventParam, CommBasicObjects::CommKBEventResult> *kbChainedEntriesEventClient;
	Smart::InputTaskTrigger<Smart::EventInputType<CommBasicObjects::CommKBEventResult>> *kbChainedEntriesEventClientInputTaskTrigger;
	KbChainedEntriesEventClientUpcallManager *kbChainedEntriesEventClientUpcallManager;
	KbChainedEntriesEventClientInputCollector *kbChainedEntriesEventClientInputCollector;
	
	// define request-ports
	
	// define input-handler
	KbChainedEntriesEventClientHandler *kbChainedEntriesEventClientHandler;
	
	// define output-ports
	Smart::IEventServerPattern<CommBasicObjects::CommKBEventParam, CommBasicObjects::CommKBEventResult, CommBasicObjects::CommVoid> *kbEventServer;
	KbEventServerWrapper *kbEventServerWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommKBEventParam, CommBasicObjects::CommKBEventResult, CommBasicObjects::CommVoid>> kbEventServerEventTestHandler;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> *kbQuery;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommKBRequest, CommBasicObjects::CommKBResponse> *kbQueryInputTaskTrigger;
	
	// define request-handlers
	KbQueryHandler *kbQueryHandler;
	
	// definitions of ComponentKBROS1InterfacesExtension
	
	// definitions of ComponentKBROSExtension
	
	// definitions of ComponentKBRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentKBExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentKBPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentKBExtension *extension);
	
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
	
	Smart::StatusCode connectKbChainedEntriesEventClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentKB* instance()
	{
		if(_componentKB == 0) {
			_componentKB = new ComponentKB();
		}
		return _componentKB;
	}
	
	static void deleteInstance() {
		if(_componentKB != 0) {
			delete _componentKB;
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
		struct Dummy_struct {
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
		} dummy;
		
		//--- upcall parameter ---
		struct KbChainedEntriesEventClientHandler_struct {
			int prescale;
		} kbChainedEntriesEventClientHandler;
		
		//--- server port parameter ---
		struct KbEventServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} kbEventServer;
		struct KbQuery_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} kbQuery;
	
		//--- client port parameter ---
		struct KbChainedEntriesEventClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} kbChainedEntriesEventClient;
		
		// -- parameters for ComponentKBROS1InterfacesExtension
		
		// -- parameters for ComponentKBROSExtension
		
		// -- parameters for ComponentKBRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentKBExtension
		
	} connections;
};
#endif
