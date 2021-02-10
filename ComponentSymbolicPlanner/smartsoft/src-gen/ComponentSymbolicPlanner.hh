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
#ifndef _COMPONENTSYMBOLICPLANNER_HH
#define _COMPONENTSYMBOLICPLANNER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentSymbolicPlannerCore.hh"

#include "ComponentSymbolicPlannerPortFactoryInterface.hh"
#include "ComponentSymbolicPlannerExtension.hh"

// forward declarations
class ComponentSymbolicPlannerPortFactoryInterface;
class ComponentSymbolicPlannerExtension;

// includes for ComponentSymbolicPlannerROS1InterfacesExtension

// includes for ComponentSymbolicPlannerROSExtension

// includes for ComponentSymbolicPlannerRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentSymbolicPlannerExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <DomainSymbolicPlanner/CommSymbolicPlannerPlan.hh>
#include <DomainSymbolicPlanner/CommSymbolicPlannerPlanACE.hh>
#include <DomainSymbolicPlanner/CommSymbolicPlannerRequest.hh>
#include <DomainSymbolicPlanner/CommSymbolicPlannerRequestACE.hh>

// include tasks
// include UpcallManagers and InputCollectors

// include input-handler(s)
// include request-handler(s)
#include "SymbolicPannerQueryHandler.hh"
// output port wrappers

// include handler
#include "CompHandler.hh"


#include "SmartStateChangeHandler.hh"

#define COMP ComponentSymbolicPlanner::instance()

class ComponentSymbolicPlanner : public ComponentSymbolicPlannerCore {
private:
	static ComponentSymbolicPlanner *_componentSymbolicPlanner;
	
	// constructor
	ComponentSymbolicPlanner();
	
	// copy-constructor
	ComponentSymbolicPlanner(const ComponentSymbolicPlanner& cc);
	
	// destructor
	~ComponentSymbolicPlanner() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentSymbolicPlannerPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentSymbolicPlannerExtension*> componentExtensionRegistry;
	
public:
	
	// define tasks
	
	// define input-ports
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	
	// define answer-ports
	Smart::IQueryServerPattern<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan> *symbolicPlannerQueryServer;
	Smart::QueryServerTaskTrigger<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan> *symbolicPlannerQueryServerInputTaskTrigger;
	
	// define request-handlers
	SymbolicPannerQueryHandler *symbolicPannerQueryHandler;
	
	// definitions of ComponentSymbolicPlannerROS1InterfacesExtension
	
	// definitions of ComponentSymbolicPlannerROSExtension
	
	// definitions of ComponentSymbolicPlannerRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentSymbolicPlannerExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentSymbolicPlannerPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentSymbolicPlannerExtension *extension);
	
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
	

	// return singleton instance
	static ComponentSymbolicPlanner* instance()
	{
		if(_componentSymbolicPlanner == 0) {
			_componentSymbolicPlanner = new ComponentSymbolicPlanner();
		}
		return _componentSymbolicPlanner;
	}
	
	static void deleteInstance() {
		if(_componentSymbolicPlanner != 0) {
			delete _componentSymbolicPlanner;
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
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct SymbolicPlannerQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} symbolicPlannerQueryServer;
	
		//--- client port parameter ---
		
		// -- parameters for ComponentSymbolicPlannerROS1InterfacesExtension
		
		// -- parameters for ComponentSymbolicPlannerROSExtension
		
		// -- parameters for ComponentSymbolicPlannerRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentSymbolicPlannerExtension
		
	} connections;
};
#endif
