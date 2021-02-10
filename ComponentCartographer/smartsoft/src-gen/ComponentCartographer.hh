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
#ifndef _COMPONENTCARTOGRAPHER_HH
#define _COMPONENTCARTOGRAPHER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentCartographerCore.hh"

#include "ComponentCartographerPortFactoryInterface.hh"
#include "ComponentCartographerExtension.hh"

// forward declarations
class ComponentCartographerPortFactoryInterface;
class ComponentCartographerExtension;

// includes for ComponentCartographerROS1InterfacesExtension

// includes for ComponentCartographerROSExtension

// includes for ComponentCartographerRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentCartographerExtension
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
#include "CartographerTask.hh"
// include UpcallManagers and InputCollectors
#include "LaserServiceInUpcallManager.hh"
#include "LaserServiceInInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
// output port wrappers
#include "GridMapPushServiceOutWrapper.hh"
#include "Localized_robot_poseWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentCartographer::instance()

class ComponentCartographer : public ComponentCartographerCore {
private:
	static ComponentCartographer *_componentCartographer;
	
	// constructor
	ComponentCartographer();
	
	// copy-constructor
	ComponentCartographer(const ComponentCartographer& cc);
	
	// destructor
	~ComponentCartographer() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentCartographerPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentCartographerExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* cartographerTaskTrigger;
	CartographerTask *cartographerTask;
	
	// define input-ports
	// InputPort laserServiceIn
	Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> *laserServiceIn;
	Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan> *laserServiceInInputTaskTrigger;
	LaserServiceInUpcallManager *laserServiceInUpcallManager;
	LaserServiceInInputCollector *laserServiceInInputCollector;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::IPushServerPattern<CommNavigationObjects::CommGridMap> *gridMapPushServiceOut;
	GridMapPushServiceOutWrapper *gridMapPushServiceOutWrapper;
	Smart::ISendClientPattern<CommBasicObjects::CommBasePositionUpdate> *localized_robot_pose;
	Localized_robot_poseWrapper *localized_robot_poseWrapper;
	
	// define answer-ports
	
	// define request-handlers
	
	// definitions of ComponentCartographerROS1InterfacesExtension
	
	// definitions of ComponentCartographerROSExtension
	
	// definitions of ComponentCartographerRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentCartographerExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentCartographerPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentCartographerExtension *extension);
	
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
	
	Smart::StatusCode connectLaserServiceIn(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectLocalized_robot_pose(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentCartographer* instance()
	{
		if(_componentCartographer == 0) {
			_componentCartographer = new ComponentCartographer();
		}
		return _componentCartographer;
	}
	
	static void deleteInstance() {
		if(_componentCartographer != 0) {
			delete _componentCartographer;
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
		struct CartographerTask_struct {
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
		} cartographerTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct GridMapPushServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} gridMapPushServiceOut;
	
		//--- client port parameter ---
		struct LaserServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} laserServiceIn;
		struct Localized_robot_pose_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} localized_robot_pose;
		
		// -- parameters for ComponentCartographerROS1InterfacesExtension
		
		// -- parameters for ComponentCartographerROSExtension
		
		// -- parameters for ComponentCartographerRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentCartographerExtension
		
	} connections;
};
#endif
