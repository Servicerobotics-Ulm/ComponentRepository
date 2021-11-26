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
#ifndef _COMPONENTEXERCISEAPPROACHCYLINDER_HH
#define _COMPONENTEXERCISEAPPROACHCYLINDER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentExerciseApproachCylinderCore.hh"

#include "ComponentExerciseApproachCylinderPortFactoryInterface.hh"
#include "ComponentExerciseApproachCylinderExtension.hh"

// forward declarations
class ComponentExerciseApproachCylinderPortFactoryInterface;
class ComponentExerciseApproachCylinderExtension;

// includes for PlainOpcUaComponentExerciseApproachCylinderExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommMobileLaserScanACE.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommNavigationVelocityACE.hh>

// include tasks
#include "RobotTask.hh"
// include UpcallManagers and InputCollectors
#include "LaserServiceInUpcallManager.hh"
#include "LaserServiceInInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
// output port wrappers
#include "NavigationVelocityServiceOutWrapper.hh"

// include handler
#include "CompHandler.hh"


#include "SmartStateChangeHandler.hh"
#include "StateActivityManager.hh"


#define COMP ComponentExerciseApproachCylinder::instance()

class ComponentExerciseApproachCylinder : public ComponentExerciseApproachCylinderCore {
private:
	static ComponentExerciseApproachCylinder *_componentExerciseApproachCylinder;
	
	// constructor
	ComponentExerciseApproachCylinder();
	
	// copy-constructor
	ComponentExerciseApproachCylinder(const ComponentExerciseApproachCylinder& cc);
	
	// destructor
	~ComponentExerciseApproachCylinder() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentExerciseApproachCylinderPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentExerciseApproachCylinderExtension*> componentExtensionRegistry;
	
public:
	
	// define tasks
	Smart::TaskTriggerSubject* robotTaskTrigger;
	RobotTask *robotTask;
	
	// define input-ports
	// InputPort LaserServiceIn
	Smart::IPushClientPattern<CommBasicObjects::CommMobileLaserScan> *laserServiceIn;
	Smart::InputTaskTrigger<CommBasicObjects::CommMobileLaserScan> *laserServiceInInputTaskTrigger;
	LaserServiceInUpcallManager *laserServiceInUpcallManager;
	LaserServiceInInputCollector *laserServiceInInputCollector;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::ISendClientPattern<CommBasicObjects::CommNavigationVelocity> *navigationVelocityServiceOut;
	NavigationVelocityServiceOutWrapper *navigationVelocityServiceOutWrapper;
	
	// define answer-ports
	
	// define request-handlers
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	StateActivityManager *stateActivityManager;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentExerciseApproachCylinderPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentExerciseApproachCylinderExtension *extension);
	
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
	Smart::StatusCode connectNavigationVelocityServiceOut(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentExerciseApproachCylinder* instance()
	{
		if(_componentExerciseApproachCylinder == 0) {
			_componentExerciseApproachCylinder = new ComponentExerciseApproachCylinder();
		}
		return _componentExerciseApproachCylinder;
	}
	
	static void deleteInstance() {
		if(_componentExerciseApproachCylinder != 0) {
			delete _componentExerciseApproachCylinder;
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
		struct RobotTask_struct {
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
		} robotTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
	
		//--- client port parameter ---
		struct LaserServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} laserServiceIn;
		struct NavigationVelocityServiceOut_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} navigationVelocityServiceOut;
		
	} connections;
};
#endif