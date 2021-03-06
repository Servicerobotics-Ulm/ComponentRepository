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
#ifndef _COMPONENTGMAPPING_HH
#define _COMPONENTGMAPPING_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentGMappingCore.hh"

#include "ComponentGMappingPortFactoryInterface.hh"
#include "ComponentGMappingExtension.hh"

// forward declarations
class ComponentGMappingPortFactoryInterface;
class ComponentGMappingExtension;

// includes for ComponentGMappingROS1InterfacesExtension

// includes for ComponentGMappingROSExtension

// includes for ComponentGMappingRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentGMappingExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommBasePositionUpdate.hh>
#include <CommBasicObjects/CommBasePositionUpdateACE.hh>
#include <CommNavigationObjects/CommGridMap.hh>
#include <CommNavigationObjects/CommGridMapACE.hh>
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>

// include tasks
#include "GMappingTask.hh"
// include UpcallManagers and InputCollectors
#include "LaserClientUpcallManager.hh"
#include "LaserClientInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
// output port wrappers
#include "BasePositionUpdateClientWrapper.hh"
#include "NewestMapPushServerWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentGMapping::instance()

class ComponentGMapping : public ComponentGMappingCore {
private:
	static ComponentGMapping *_componentGMapping;
	
	// constructor
	ComponentGMapping();
	
	// copy-constructor
	ComponentGMapping(const ComponentGMapping& cc);
	
	// destructor
	~ComponentGMapping() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentGMappingPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentGMappingExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* gMappingTaskTrigger;
	GMappingTask *gMappingTask;
	
	// define input-ports
	// InputPort laserClient
	Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> *laserClient;
	Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan> *laserClientInputTaskTrigger;
	LaserClientUpcallManager *laserClientUpcallManager;
	LaserClientInputCollector *laserClientInputCollector;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::ISendClientPattern<CommBasicObjects::CommBasePositionUpdate> *basePositionUpdateClient;
	BasePositionUpdateClientWrapper *basePositionUpdateClientWrapper;
	Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> *newestMapPushServer;
	NewestMapPushServerWrapper *newestMapPushServerWrapper;
	
	// define answer-ports
	
	// define request-handlers
	
	// definitions of ComponentGMappingROS1InterfacesExtension
	
	// definitions of ComponentGMappingROSExtension
	
	// definitions of ComponentGMappingRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentGMappingExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentGMappingPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentGMappingExtension *extension);
	
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
	
	Smart::StatusCode connectBasePositionUpdateClient(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectLaserClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentGMapping* instance()
	{
		if(_componentGMapping == 0) {
			_componentGMapping = new ComponentGMapping();
		}
		return _componentGMapping;
	}
	
	static void deleteInstance() {
		if(_componentGMapping != 0) {
			delete _componentGMapping;
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
		struct GMappingTask_struct {
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
		} gMappingTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct NewestMapPushServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} newestMapPushServer;
	
		//--- client port parameter ---
		struct BasePositionUpdateClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} basePositionUpdateClient;
		struct LaserClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} laserClient;
		
		// -- parameters for ComponentGMappingROS1InterfacesExtension
		
		// -- parameters for ComponentGMappingROSExtension
		
		// -- parameters for ComponentGMappingRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentGMappingExtension
		
	} connections;
};
#endif
