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
#ifndef _COMPONENTWEBOTSURSERVER_HH
#define _COMPONENTWEBOTSURSERVER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentWebotsURServerCore.hh"

#include "ComponentWebotsURServerPortFactoryInterface.hh"
#include "ComponentWebotsURServerExtension.hh"

// forward declarations
class ComponentWebotsURServerPortFactoryInterface;
class ComponentWebotsURServerExtension;

// includes for PlainOpcUaComponentWebotsURServerExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommDigitalInputEventParameter.hh>
#include <CommBasicObjects/CommDigitalInputEventParameterACE.hh>
#include <CommBasicObjects/CommDigitalInputEventResult.hh>
#include <CommBasicObjects/CommDigitalInputEventResultACE.hh>
#include <CommBasicObjects/CommDigitalInputEventState.hh>
#include <CommBasicObjects/CommDigitalInputEventStateACE.hh>
#include <CommBasicObjects/CommIOValues.hh>
#include <CommBasicObjects/CommIOValuesACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameter.hh>
#include <CommManipulatorObjects/CommManipulatorEventParameterACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventResult.hh>
#include <CommManipulatorObjects/CommManipulatorEventResultACE.hh>
#include <CommManipulatorObjects/CommManipulatorEventState.hh>
#include <CommManipulatorObjects/CommManipulatorEventStateACE.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectoryACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorPrograms.hh>
#include <CommManipulatorObjects/CommMobileManipulatorProgramsACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "PoseUpdateActivity.hh"
// include UpcallManagers and InputCollectors
#include "BaseStateServiceInUpcallManager.hh"
#include "BaseStateServiceInInputCollector.hh"
#include "TrajectorySendServerUpcallManager.hh"
#include "TrajectorySendServerInputCollector.hh"

// include input-handler(s)
#include "TrajectorySendServerHandler.hh"
// include request-handler(s)
#include "IoQueryServerHandler.hh"
#include "PoseQueryServerHandler.hh"
#include "ProgramQueryHandler.hh"
// output port wrappers
#include "PosePushServerWrapper.hh"
#include "ManipulatorEventServiceOutWrapper.hh"
#include "IoEventServerWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentWebotsURServer::instance()

class ComponentWebotsURServer : public ComponentWebotsURServerCore {
private:
	static ComponentWebotsURServer *_componentWebotsURServer;
	
	// constructor
	ComponentWebotsURServer();
	
	// copy-constructor
	ComponentWebotsURServer(const ComponentWebotsURServer& cc);
	
	// destructor
	~ComponentWebotsURServer() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentWebotsURServerPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentWebotsURServerExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* poseUpdateActivityTrigger;
	PoseUpdateActivity *poseUpdateActivity;
	
	// define input-ports
	// InputPort baseStateServiceIn
	Smart::IPushClientPattern<CommBasicObjects::CommBaseState> *baseStateServiceIn;
	Smart::InputTaskTrigger<CommBasicObjects::CommBaseState> *baseStateServiceInInputTaskTrigger;
	BaseStateServiceInUpcallManager *baseStateServiceInUpcallManager;
	BaseStateServiceInInputCollector *baseStateServiceInInputCollector;
	// InputPort trajectorySendServer
	Smart::ISendServerPattern<CommManipulatorObjects::CommManipulatorTrajectory> *trajectorySendServer;
	Smart::InputTaskTrigger<CommManipulatorObjects::CommManipulatorTrajectory> *trajectorySendServerInputTaskTrigger;
	TrajectorySendServerUpcallManager *trajectorySendServerUpcallManager;
	TrajectorySendServerInputCollector *trajectorySendServerInputCollector;
	
	// define request-ports
	
	// define input-handler
	TrajectorySendServerHandler *trajectorySendServerHandler;
	
	// define output-ports
	Smart::IEventServerPattern<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState> *ioEventServer;
	IoEventServerWrapper *ioEventServerWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommDigitalInputEventParameter, CommBasicObjects::CommDigitalInputEventResult, CommBasicObjects::CommDigitalInputEventState>> ioEventServerEventTestHandler;
	Smart::IEventServerPattern<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState> *manipulatorEventServiceOut;
	ManipulatorEventServiceOutWrapper *manipulatorEventServiceOutWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommManipulatorObjects::CommManipulatorEventParameter, CommManipulatorObjects::CommManipulatorEventResult, CommManipulatorObjects::CommManipulatorEventState>> manipulatorEventServiceOutEventTestHandler;
	Smart::IPushServerPattern<CommManipulatorObjects::CommMobileManipulatorState> *posePushServer;
	PosePushServerWrapper *posePushServerWrapper;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> *ioQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommIOValues, CommBasicObjects::CommIOValues> *ioQueryServerInputTaskTrigger;
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> *poseQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> *poseQueryServerInputTaskTrigger;
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms> *programQuery;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorPrograms> *programQueryInputTaskTrigger;
	
	// define request-handlers
	IoQueryServerHandler *ioQueryServerHandler;
	PoseQueryServerHandler *poseQueryServerHandler;
	ProgramQueryHandler *programQueryHandler;
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentWebotsURServerPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentWebotsURServerExtension *extension);
	
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
	
	Smart::StatusCode connectBaseStateServiceIn(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentWebotsURServer* instance()
	{
		if(_componentWebotsURServer == 0) {
			_componentWebotsURServer = new ComponentWebotsURServer();
		}
		return _componentWebotsURServer;
	}
	
	static void deleteInstance() {
		if(_componentWebotsURServer != 0) {
			delete _componentWebotsURServer;
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
		struct PoseUpdateActivity_struct {
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
		} poseUpdateActivity;
		
		//--- upcall parameter ---
		struct TrajectorySendServerHandler_struct {
			int prescale;
		} trajectorySendServerHandler;
		
		//--- server port parameter ---
		struct IoEventServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} ioEventServer;
		struct IoQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} ioQueryServer;
		struct ManipulatorEventServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} manipulatorEventServiceOut;
		struct PosePushServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} posePushServer;
		struct PoseQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} poseQueryServer;
		struct ProgramQuery_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} programQuery;
		struct TrajectorySendServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} trajectorySendServer;
	
		//--- client port parameter ---
		struct BaseStateServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} baseStateServiceIn;
		
	} connections;
};
#endif
