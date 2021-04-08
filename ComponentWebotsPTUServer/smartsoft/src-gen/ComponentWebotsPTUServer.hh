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
#ifndef _COMPONENTWEBOTSPTUSERVER_HH
#define _COMPONENTWEBOTSPTUSERVER_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentWebotsPTUServerCore.hh"

#include "ComponentWebotsPTUServerPortFactoryInterface.hh"
#include "ComponentWebotsPTUServerExtension.hh"

// forward declarations
class ComponentWebotsPTUServerPortFactoryInterface;
class ComponentWebotsPTUServerExtension;

// includes for ComponentWebotsPTUServerROS1InterfacesExtension

// includes for ComponentWebotsPTUServerROSExtension

// includes for ComponentWebotsPTUServerRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentWebotsPTUServerExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <CommBasicObjects/CommDevicePoseState.hh>
#include <CommBasicObjects/CommDevicePoseStateACE.hh>
#include <DomainPTU/CommMobilePTUState.hh>
#include <DomainPTU/CommMobilePTUStateACE.hh>
#include <DomainPTU/CommPTUGoalEventParameter.hh>
#include <DomainPTU/CommPTUGoalEventParameterACE.hh>
#include <DomainPTU/CommPTUGoalEventResult.hh>
#include <DomainPTU/CommPTUGoalEventResultACE.hh>
#include <DomainPTU/CommPTUMoveRequest.hh>
#include <DomainPTU/CommPTUMoveRequestACE.hh>
#include <DomainPTU/CommPTUMoveResponse.hh>
#include <DomainPTU/CommPTUMoveResponseACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>
#include <DomainPTU/PTUGoalEventState.hh>
#include <DomainPTU/PTUGoalEventStateACE.hh>

// include tasks
#include "BaseStateTask.hh"
#include "DevicePoseStateTask.hh"
#include "PtuTask.hh"
#include "WebotsTask.hh"
// include UpcallManagers and InputCollectors
#include "BaseStateClientUpcallManager.hh"
#include "BaseStateClientInputCollector.hh"
#include "MoveSendServerUpcallManager.hh"
#include "MoveSendServerInputCollector.hh"

// include input-handler(s)
#include "MoveSendHandler.hh"
// include request-handler(s)
#include "MovePTUQueryServiceAnswHandler.hh"
#include "PTUStateQueryServiceAnswHandler.hh"
// output port wrappers
#include "DevicePoseStateServerWrapper.hh"
#include "GoalEventServerWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentWebotsPTUServer::instance()

class ComponentWebotsPTUServer : public ComponentWebotsPTUServerCore {
private:
	static ComponentWebotsPTUServer *_componentWebotsPTUServer;
	
	// constructor
	ComponentWebotsPTUServer();
	
	// copy-constructor
	ComponentWebotsPTUServer(const ComponentWebotsPTUServer& cc);
	
	// destructor
	~ComponentWebotsPTUServer() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentWebotsPTUServerPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentWebotsPTUServerExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* baseStateTaskTrigger;
	BaseStateTask *baseStateTask;
	Smart::TaskTriggerSubject* devicePoseStateTaskTrigger;
	DevicePoseStateTask *devicePoseStateTask;
	Smart::TaskTriggerSubject* ptuTaskTrigger;
	PtuTask *ptuTask;
	Smart::TaskTriggerSubject* webotsTaskTrigger;
	WebotsTask *webotsTask;
	
	// define input-ports
	// InputPort baseStateClient
	Smart::IPushClientPattern<CommBasicObjects::CommBaseState> *baseStateClient;
	Smart::InputTaskTrigger<CommBasicObjects::CommBaseState> *baseStateClientInputTaskTrigger;
	BaseStateClientUpcallManager *baseStateClientUpcallManager;
	BaseStateClientInputCollector *baseStateClientInputCollector;
	// InputPort moveSendServer
	Smart::ISendServerPattern<DomainPTU::CommPTUMoveRequest> *moveSendServer;
	Smart::InputTaskTrigger<DomainPTU::CommPTUMoveRequest> *moveSendServerInputTaskTrigger;
	MoveSendServerUpcallManager *moveSendServerUpcallManager;
	MoveSendServerInputCollector *moveSendServerInputCollector;
	
	// define request-ports
	Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState> *baseStateQueryClient;
	
	// define input-handler
	MoveSendHandler *moveSendHandler;
	
	// define output-ports
	Smart::IPushServerPattern<CommBasicObjects::CommDevicePoseState> *devicePoseStateServer;
	DevicePoseStateServerWrapper *devicePoseStateServerWrapper;
	Smart::IEventServerPattern<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState> *goalEventServer;
	GoalEventServerWrapper *goalEventServerWrapper;
	std::shared_ptr<Smart::IEventTestHandler<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState>> goalEventServerEventTestHandler;
	
	// define answer-ports
	Smart::IQueryServerPattern<DomainPTU::CommPTUMoveRequest, DomainPTU::CommPTUMoveResponse> *moveQueryServer;
	Smart::QueryServerTaskTrigger<DomainPTU::CommPTUMoveRequest, DomainPTU::CommPTUMoveResponse> *moveQueryServerInputTaskTrigger;
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainPTU::CommMobilePTUState> *stateQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, DomainPTU::CommMobilePTUState> *stateQueryServerInputTaskTrigger;
	
	// define request-handlers
	MovePTUQueryServiceAnswHandler *movePTUQueryServiceAnswHandler;
	PTUStateQueryServiceAnswHandler *pTUStateQueryServiceAnswHandler;
	
	// definitions of ComponentWebotsPTUServerROS1InterfacesExtension
	
	// definitions of ComponentWebotsPTUServerROSExtension
	
	// definitions of ComponentWebotsPTUServerRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentWebotsPTUServerExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentWebotsPTUServerPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentWebotsPTUServerExtension *extension);
	
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
	
	Smart::StatusCode connectBaseStateClient(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectBaseStateQueryClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentWebotsPTUServer* instance()
	{
		if(_componentWebotsPTUServer == 0) {
			_componentWebotsPTUServer = new ComponentWebotsPTUServer();
		}
		return _componentWebotsPTUServer;
	}
	
	static void deleteInstance() {
		if(_componentWebotsPTUServer != 0) {
			delete _componentWebotsPTUServer;
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
		struct BaseStateTask_struct {
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
		} baseStateTask;
		struct DevicePoseStateTask_struct {
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
		} devicePoseStateTask;
		struct PtuTask_struct {
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
		} ptuTask;
		struct WebotsTask_struct {
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
		} webotsTask;
		
		//--- upcall parameter ---
		struct MoveSendHandler_struct {
			int prescale;
		} moveSendHandler;
		
		//--- server port parameter ---
		struct DevicePoseStateServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} devicePoseStateServer;
		struct GoalEventServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} goalEventServer;
		struct MoveQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} moveQueryServer;
		struct MoveSendServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} moveSendServer;
		struct StateQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} stateQueryServer;
	
		//--- client port parameter ---
		struct BaseStateClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} baseStateClient;
		struct BaseStateQueryClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} baseStateQueryClient;
		
		// -- parameters for ComponentWebotsPTUServerROS1InterfacesExtension
		
		// -- parameters for ComponentWebotsPTUServerROSExtension
		
		// -- parameters for ComponentWebotsPTUServerRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentWebotsPTUServerExtension
		
	} connections;
};
#endif
