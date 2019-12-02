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
#ifndef _COMPONENTLASERS300SERVER_HH
#define _COMPONENTLASERS300SERVER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentLaserS300ServerCore.hh"

#include "ComponentLaserS300ServerPortFactoryInterface.hh"
#include "ComponentLaserS300ServerExtension.hh"

// forward declarations
class ComponentLaserS300ServerPortFactoryInterface;
class ComponentLaserS300ServerExtension;

// includes for OpcUaBackendComponentGeneratorExtension

// includes for ComponentLaserS300ServerROSExtension

// includes for PlainOpcUaComponentLaserS300ServerExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommLaserSafetyEventParam.hh>
#include <CommBasicObjects/CommLaserSafetyEventParamACE.hh>
#include <CommBasicObjects/CommLaserSafetyEventState.hh>
#include <CommBasicObjects/CommLaserSafetyEventStateACE.hh>
#include <CommBasicObjects/CommLaserSafetyField.hh>
#include <CommBasicObjects/CommLaserSafetyFieldACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "LaserTask.hh"
// include UpcallManagers
#include "BaseTimedClientUpcallManager.hh"

// include input-handler(s)
// include request-handler(s)
#include "LaserQueryServiceAnswHandler.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentLaserS300Server::instance()

class ComponentLaserS300Server : public ComponentLaserS300ServerCore {
private:
	static ComponentLaserS300Server *_componentLaserS300Server;
	
	// constructor
	ComponentLaserS300Server();
	
	// copy-constructor
	ComponentLaserS300Server(const ComponentLaserS300Server& cc);
	
	// destructor
	~ComponentLaserS300Server() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentLaserS300ServerPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentLaserS300ServerExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* laserTaskTrigger;
	LaserTask *laserTask;
	
	// define input-ports
	// InputPort baseTimedClient
	Smart::IPushClientPattern<CommBasicObjects::CommBaseState> *baseTimedClient;
	Smart::InputTaskTrigger<CommBasicObjects::CommBaseState> *baseTimedClientInputTaskTrigger;
	BaseTimedClientUpcallManager *baseTimedClientUpcallManager;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> *laserPushNewestServer;
	Smart::IEventServerPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState> *safetyfieldEventServer;
	std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState>> safetyfieldEventServerEventTestHandler;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan> *laserQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan> *laserQueryServerInputTaskTrigger;
	
	// define request-handlers
	LaserQueryServiceAnswHandler *laserQueryServiceAnswHandler;
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of ComponentLaserS300ServerROSExtension
	
	// definitions of PlainOpcUaComponentLaserS300ServerExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentLaserS300ServerPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentLaserS300ServerExtension *extension);
	
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
	
	Smart::StatusCode connectBaseTimedClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentLaserS300Server* instance()
	{
		if(_componentLaserS300Server == 0) {
			_componentLaserS300Server = new ComponentLaserS300Server();
		}
		return _componentLaserS300Server;
	}
	
	static void deleteInstance() {
		if(_componentLaserS300Server != 0) {
			delete _componentLaserS300Server;
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
		struct LaserTask_struct {
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
		} laserTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct LaserPushNewestServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} laserPushNewestServer;
		struct LaserQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} laserQueryServer;
		struct SafetyfieldEventServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} safetyfieldEventServer;
	
		//--- client port parameter ---
		struct BaseTimedClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} baseTimedClient;
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for ComponentLaserS300ServerROSExtension
		
		// -- parameters for PlainOpcUaComponentLaserS300ServerExtension
		
	} connections;
};
#endif
