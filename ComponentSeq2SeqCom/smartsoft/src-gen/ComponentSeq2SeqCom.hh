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
#ifndef _COMPONENTSEQ2SEQCOM_HH
#define _COMPONENTSEQ2SEQCOM_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentSeq2SeqComCore.hh"

#include "ComponentSeq2SeqComPortFactoryInterface.hh"
#include "ComponentSeq2SeqComExtension.hh"

// forward declarations
class ComponentSeq2SeqComPortFactoryInterface;
class ComponentSeq2SeqComExtension;

// includes for ComponentSeq2SeqComROS1InterfacesExtension

// includes for ComponentSeq2SeqComROSExtension

// includes for ComponentSeq2SeqComRestInterfacesExtension

// includes for OpcUaBackendComponentGeneratorExtension

// includes for PlainOpcUaComponentSeq2SeqComExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommTaskEventState.hh>
#include <CommBasicObjects/CommTaskEventStateACE.hh>
#include <CommBasicObjects/CommTaskMessage.hh>
#include <CommBasicObjects/CommTaskMessageACE.hh>

// include tasks
#include "DebugTask.hh"
// include UpcallManagers and InputCollectors
#include "TaskSendInUpcallManager.hh"
#include "TaskSendInInputCollector.hh"

// include input-handler(s)
#include "CommandHandler.hh"
// include request-handler(s)
// output port wrappers
#include "TaskSendOutWrapper.hh"
#include "TaskEventOutWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentSeq2SeqCom::instance()

class ComponentSeq2SeqCom : public ComponentSeq2SeqComCore {
private:
	static ComponentSeq2SeqCom *_componentSeq2SeqCom;
	
	// constructor
	ComponentSeq2SeqCom();
	
	// copy-constructor
	ComponentSeq2SeqCom(const ComponentSeq2SeqCom& cc);
	
	// destructor
	~ComponentSeq2SeqCom() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentSeq2SeqComPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentSeq2SeqComExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* debugTaskTrigger;
	DebugTask *debugTask;
	
	// define input-ports
	// InputPort TaskSendIn
	Smart::ISendServerPattern<CommBasicObjects::CommTaskMessage> *taskSendIn;
	Smart::InputTaskTrigger<CommBasicObjects::CommTaskMessage> *taskSendInInputTaskTrigger;
	TaskSendInUpcallManager *taskSendInUpcallManager;
	TaskSendInInputCollector *taskSendInInputCollector;
	
	// define request-ports
	
	// define input-handler
	CommandHandler *commandHandler;
	
	// define output-ports
	Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> *taskEventOut;
	TaskEventOutWrapper *taskEventOutWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState>> taskEventOutEventTestHandler;
	Smart::ISendClientPattern<CommBasicObjects::CommTaskMessage> *taskSendOut;
	TaskSendOutWrapper *taskSendOutWrapper;
	
	// define answer-ports
	
	// define request-handlers
	
	// definitions of ComponentSeq2SeqComROS1InterfacesExtension
	
	// definitions of ComponentSeq2SeqComROSExtension
	
	// definitions of ComponentSeq2SeqComRestInterfacesExtension
	
	// definitions of OpcUaBackendComponentGeneratorExtension
	
	// definitions of PlainOpcUaComponentSeq2SeqComExtension
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentSeq2SeqComPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentSeq2SeqComExtension *extension);
	
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
	
	Smart::StatusCode connectTaskSendOut(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentSeq2SeqCom* instance()
	{
		if(_componentSeq2SeqCom == 0) {
			_componentSeq2SeqCom = new ComponentSeq2SeqCom();
		}
		return _componentSeq2SeqCom;
	}
	
	static void deleteInstance() {
		if(_componentSeq2SeqCom != 0) {
			delete _componentSeq2SeqCom;
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
		struct DebugTask_struct {
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
		} debugTask;
		
		//--- upcall parameter ---
		struct CommandHandler_struct {
			int prescale;
		} commandHandler;
		
		//--- server port parameter ---
		struct TaskEventOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} taskEventOut;
		struct TaskSendIn_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} taskSendIn;
	
		//--- client port parameter ---
		struct TaskSendOut_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} taskSendOut;
		
		// -- parameters for ComponentSeq2SeqComROS1InterfacesExtension
		
		// -- parameters for ComponentSeq2SeqComROSExtension
		
		// -- parameters for ComponentSeq2SeqComRestInterfacesExtension
		
		// -- parameters for OpcUaBackendComponentGeneratorExtension
		
		// -- parameters for PlainOpcUaComponentSeq2SeqComExtension
		
	} connections;
};
#endif
