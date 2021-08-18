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
#ifndef _COMPONENTRACKDETECTION_HH
#define _COMPONENTRACKDETECTION_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentRackDetectionCore.hh"

#include "ComponentRackDetectionPortFactoryInterface.hh"
#include "ComponentRackDetectionExtension.hh"

// forward declarations
class ComponentRackDetectionPortFactoryInterface;
class ComponentRackDetectionExtension;

// includes for PlainOpcUaComponentRackDetectionExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironment.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironmentACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventResult.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventResultACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventState.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventStateACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionId.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionIdACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectPropertiesACE.hh>
#include <DomainVision/CommRGBDImage.hh>
#include <DomainVision/CommRGBDImageACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "RackDetectionTask.hh"
// include UpcallManagers and InputCollectors

// include input-handler(s)
// include request-handler(s)
#include "EnvironmentQueryServiceAnswHandler.hh"
#include "ObjectQueryServiceAnswHandler.hh"
// output port wrappers
#include "ObjectEventServerWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"
#include "StateActivityManager.hh"


#define COMP ComponentRackDetection::instance()

class ComponentRackDetection : public ComponentRackDetectionCore {
private:
	static ComponentRackDetection *_componentRackDetection;
	
	// constructor
	ComponentRackDetection();
	
	// copy-constructor
	ComponentRackDetection(const ComponentRackDetection& cc);
	
	// destructor
	~ComponentRackDetection() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentRackDetectionPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentRackDetectionExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* rackDetectionTaskTrigger;
	RackDetectionTask *rackDetectionTask;
	
	// define input-ports
	
	// define request-ports
	Smart::IQueryClientPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> *kinectQueryClient;
	
	// define input-handler
	
	// define output-ports
	Smart::IEventServerPattern<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState> *objectEventServer;
	ObjectEventServerWrapper *objectEventServerWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommVoid, CommObjectRecognitionObjects::CommObjectRecognitionEventResult, CommObjectRecognitionObjects::CommObjectRecognitionEventState>> objectEventServerEventTestHandler;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> *environmentQueryServer;
	Smart::QueryServerTaskTrigger<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> *environmentQueryServerInputTaskTrigger;
	Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> *objectPropertyQueryServer;
	Smart::QueryServerTaskTrigger<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> *objectPropertyQueryServerInputTaskTrigger;
	
	// define request-handlers
	EnvironmentQueryServiceAnswHandler *environmentQueryServiceAnswHandler;
	ObjectQueryServiceAnswHandler *objectQueryServiceAnswHandler;
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	StateActivityManager *stateActivityManager;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentRackDetectionPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentRackDetectionExtension *extension);
	
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
	
	Smart::StatusCode connectKinectQueryClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentRackDetection* instance()
	{
		if(_componentRackDetection == 0) {
			_componentRackDetection = new ComponentRackDetection();
		}
		return _componentRackDetection;
	}
	
	static void deleteInstance() {
		if(_componentRackDetection != 0) {
			delete _componentRackDetection;
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
		struct RackDetectionTask_struct {
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
		} rackDetectionTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct EnvironmentQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} environmentQueryServer;
		struct ObjectEventServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} objectEventServer;
		struct ObjectPropertyQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} objectPropertyQueryServer;
	
		//--- client port parameter ---
		struct KinectQueryClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} kinectQueryClient;
		
	} connections;
};
#endif
