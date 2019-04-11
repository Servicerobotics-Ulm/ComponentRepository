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
#ifndef _COMPONENTKEYBOARDNAVIGATION_HH
#define _COMPONENTKEYBOARDNAVIGATION_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentKeyboardNavigationCore.hh"

#include "ComponentKeyboardNavigationPortFactoryInterface.hh"
#include "ComponentKeyboardNavigationExtension.hh"

// forward declarations
class ComponentKeyboardNavigationPortFactoryInterface;
class ComponentKeyboardNavigationExtension;

// includes for ComponentKeyboardNavigationROSExtension

// includes for PlainOpcUaComponentKeyboardNavigationExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommNavigationVelocityACE.hh>

// include tasks
#include "KeyboardInputTask.hh"
// include UpcallManagers

// include input-handler
// include input-handler

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentKeyboardNavigation::instance()

class ComponentKeyboardNavigation : public ComponentKeyboardNavigationCore {
private:
	static ComponentKeyboardNavigation *_componentKeyboardNavigation;
	
	// constructor
	ComponentKeyboardNavigation();
	
	// copy-constructor
	ComponentKeyboardNavigation(const ComponentKeyboardNavigation& cc);
	
	// destructor
	~ComponentKeyboardNavigation() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentKeyboardNavigationPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentKeyboardNavigationExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* keyboardInputTaskTrigger;
	KeyboardInputTask *keyboardInputTask;
	
	// define input-ports
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> *navVelSendClient;
	
	// define answer-ports
	
	// define request-handlers
	
	// definitions of ComponentKeyboardNavigationROSExtension
	
	// definitions of PlainOpcUaComponentKeyboardNavigationExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentKeyboardNavigationPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentKeyboardNavigationExtension *extension);
	
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
	
	Smart::StatusCode connectNavVelSendClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentKeyboardNavigation* instance()
	{
		if(_componentKeyboardNavigation == 0) {
			_componentKeyboardNavigation = new ComponentKeyboardNavigation();
		}
		return _componentKeyboardNavigation;
	}
	
	static void deleteInstance() {
		if(_componentKeyboardNavigation != 0) {
			delete _componentKeyboardNavigation;
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
		struct KeyboardInputTask_struct {
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
		} keyboardInputTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
	
		//--- client port parameter ---
		struct NavVelSendClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} navVelSendClient;
		
		// -- parameters for ComponentKeyboardNavigationROSExtension
		
		// -- parameters for PlainOpcUaComponentKeyboardNavigationExtension
		
	} connections;
};
#endif
