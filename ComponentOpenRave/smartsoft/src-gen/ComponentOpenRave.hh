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
#ifndef _COMPONENTOPENRAVE_HH
#define _COMPONENTOPENRAVE_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentOpenRaveCore.hh"

#include "ComponentOpenRavePortFactoryInterface.hh"
#include "ComponentOpenRaveExtension.hh"

// forward declarations
class ComponentOpenRavePortFactoryInterface;
class ComponentOpenRaveExtension;

// includes for PlainOpcUaComponentOpenRaveExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommManipulatorObjects/CommGripperState.hh>
#include <CommManipulatorObjects/CommGripperStateACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventParameter.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventParameterACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventResult.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventResultACE.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventState.hh>
#include <CommManipulationPlannerObjects/CommManipulationPlannerEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameter.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameterACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventResult.hh>
#include <CommManipulatorObjects/CommManipulatorEventResultACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventState.hh>
#include <CommManipulatorObjects/CommManipulatorEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectoryACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironment.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEnvironmentACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionId.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionIdACE.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectPropertiesACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "DemonstrationTask.hh"
#include "EventActivity.hh"
#include "TrajectorySampling.hh"
// include UpcallManagers and InputCollectors
#include "GripperEventServiceInUpcallManager.hh"
#include "GripperEventServiceInInputCollector.hh"
#include "GripperStateServiceInUpcallManager.hh"
#include "GripperStateServiceInInputCollector.hh"
#include "ManipulatorEventServiceInUpcallManager.hh"
#include "ManipulatorEventServiceInInputCollector.hh"
#include "MobileManipulatorStateServiceInUpcallManager.hh"
#include "MobileManipulatorStateServiceInInputCollector.hh"

// include input-handler(s)
#include "GripperEventServiceInHandler.hh"
#include "ManipulatorEventServiceInHandler.hh"
// include request-handler(s)
#include "ObjectQueryServiceAnswHandler.hh"
// output port wrappers
#include "SendTrajectoryOutWrapper.hh"
#include "PlanningEventServiceOutWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentOpenRave::instance()

class ComponentOpenRave : public ComponentOpenRaveCore {
private:
	static ComponentOpenRave *_componentOpenRave;
	
	// constructor
	ComponentOpenRave();
	
	// copy-constructor
	ComponentOpenRave(const ComponentOpenRave& cc);
	
	// destructor
	~ComponentOpenRave() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentOpenRavePortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentOpenRaveExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* demonstrationTaskTrigger;
	DemonstrationTask *demonstrationTask;
	Smart::TaskTriggerSubject* eventActivityTrigger;
	EventActivity *eventActivity;
	Smart::TaskTriggerSubject* trajectorySamplingTrigger;
	TrajectorySampling *trajectorySampling;
	
	// define input-ports
	// InputPort GripperEventServiceIn
	Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> *gripperEventServiceIn;
	Smart::InputTaskTrigger<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>> *gripperEventServiceInInputTaskTrigger;
	GripperEventServiceInUpcallManager *gripperEventServiceInUpcallManager;
	GripperEventServiceInInputCollector *gripperEventServiceInInputCollector;
	// InputPort GripperStateServiceIn
	Smart::IPushClientPattern<CommManipulatorObjects::CommGripperState> *gripperStateServiceIn;
	Smart::InputTaskTrigger<CommManipulatorObjects::CommGripperState> *gripperStateServiceInInputTaskTrigger;
	GripperStateServiceInUpcallManager *gripperStateServiceInUpcallManager;
	GripperStateServiceInInputCollector *gripperStateServiceInInputCollector;
	// InputPort ManipulatorEventServiceIn
	Smart::IEventClientPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult> *manipulatorEventServiceIn;
	Smart::InputTaskTrigger<Smart::EventInputType<CommManipulatorObjects::CommManipulatorEventResult>> *manipulatorEventServiceInInputTaskTrigger;
	ManipulatorEventServiceInUpcallManager *manipulatorEventServiceInUpcallManager;
	ManipulatorEventServiceInInputCollector *manipulatorEventServiceInInputCollector;
	// InputPort MobileManipulatorStateServiceIn
	Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> *mobileManipulatorStateServiceIn;
	Smart::InputTaskTrigger<CommManipulatorObjects::CommMobileManipulatorState> *mobileManipulatorStateServiceInInputTaskTrigger;
	MobileManipulatorStateServiceInUpcallManager *mobileManipulatorStateServiceInUpcallManager;
	MobileManipulatorStateServiceInInputCollector *mobileManipulatorStateServiceInInputCollector;
	
	// define request-ports
	Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionEnvironment> *environmentQueryServiceReq;
	Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommGripperState> *gripperStateQueryServiceReq;
	Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> *mobileManipulatorStateQueryServiceReq;
	Smart::IQueryClientPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> *objectQueryServiceReq;
	
	// define input-handler
	GripperEventServiceInHandler *gripperEventServiceInHandler;
	ManipulatorEventServiceInHandler *manipulatorEventServiceInHandler;
	
	// define output-ports
	Smart::IEventServerPattern<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState> *planningEventServiceOut;
	PlanningEventServiceOutWrapper *planningEventServiceOutWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState>> planningEventServiceOutEventTestHandler;
	Smart::ISendClientPattern<CommManipulatorObjects::CommManipulatorTrajectory> *sendTrajectoryOut;
	SendTrajectoryOutWrapper *sendTrajectoryOutWrapper;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> *objectQueryServiceAnsw;
	Smart::QueryServerTaskTrigger<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties> *objectQueryServiceAnswInputTaskTrigger;
	
	// define request-handlers
	ObjectQueryServiceAnswHandler *objectQueryServiceAnswHandler;
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentOpenRavePortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentOpenRaveExtension *extension);
	
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
	
	Smart::StatusCode connectEnvironmentQueryServiceReq(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectGripperEventServiceIn(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectGripperStateQueryServiceReq(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectGripperStateServiceIn(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectManipulatorEventServiceIn(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectMobileManipulatorStateQueryServiceReq(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectMobileManipulatorStateServiceIn(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectObjectQueryServiceReq(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectSendTrajectoryOut(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentOpenRave* instance()
	{
		if(_componentOpenRave == 0) {
			_componentOpenRave = new ComponentOpenRave();
		}
		return _componentOpenRave;
	}
	
	static void deleteInstance() {
		if(_componentOpenRave != 0) {
			delete _componentOpenRave;
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
		struct DemonstrationTask_struct {
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
		} demonstrationTask;
		struct EventActivity_struct {
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
		} eventActivity;
		struct TrajectorySampling_struct {
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
		} trajectorySampling;
		
		//--- upcall parameter ---
		struct GripperEventServiceInHandler_struct {
			int prescale;
		} gripperEventServiceInHandler;
		struct ManipulatorEventServiceInHandler_struct {
			int prescale;
		} manipulatorEventServiceInHandler;
		
		//--- server port parameter ---
		struct ObjectQueryServiceAnsw_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} objectQueryServiceAnsw;
		struct PlanningEventServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} planningEventServiceOut;
	
		//--- client port parameter ---
		struct EnvironmentQueryServiceReq_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} environmentQueryServiceReq;
		struct GripperEventServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} gripperEventServiceIn;
		struct GripperStateQueryServiceReq_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} gripperStateQueryServiceReq;
		struct GripperStateServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} gripperStateServiceIn;
		struct ManipulatorEventServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} manipulatorEventServiceIn;
		struct MobileManipulatorStateQueryServiceReq_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} mobileManipulatorStateQueryServiceReq;
		struct MobileManipulatorStateServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} mobileManipulatorStateServiceIn;
		struct ObjectQueryServiceReq_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} objectQueryServiceReq;
		struct SendTrajectoryOut_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} sendTrajectoryOut;
		
	} connections;
};
#endif
