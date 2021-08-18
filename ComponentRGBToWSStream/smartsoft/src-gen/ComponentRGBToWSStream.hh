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
#ifndef _COMPONENTRGBTOWSSTREAM_HH
#define _COMPONENTRGBTOWSSTREAM_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentRGBToWSStreamCore.hh"

#include "ComponentRGBToWSStreamPortFactoryInterface.hh"
#include "ComponentRGBToWSStreamExtension.hh"

// forward declarations
class ComponentRGBToWSStreamPortFactoryInterface;
class ComponentRGBToWSStreamExtension;

// includes for PlainOpcUaComponentRGBToWSStreamExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>

// include tasks
#include "CameraTask.hh"
#include "ServerTask.hh"
// include UpcallManagers and InputCollectors
#include "VideoImageClientUpcallManager.hh"
#include "VideoImageClientInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
// output port wrappers

// include handler
#include "CompHandler.hh"


#include "SmartStateChangeHandler.hh"
#include "StateActivityManager.hh"


#define COMP ComponentRGBToWSStream::instance()

class ComponentRGBToWSStream : public ComponentRGBToWSStreamCore {
private:
	static ComponentRGBToWSStream *_componentRGBToWSStream;
	
	// constructor
	ComponentRGBToWSStream();
	
	// copy-constructor
	ComponentRGBToWSStream(const ComponentRGBToWSStream& cc);
	
	// destructor
	~ComponentRGBToWSStream() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentRGBToWSStreamPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentRGBToWSStreamExtension*> componentExtensionRegistry;
	
public:
	
	// define tasks
	Smart::TaskTriggerSubject* cameraTaskTrigger;
	CameraTask *cameraTask;
	Smart::TaskTriggerSubject* serverTaskTrigger;
	ServerTask *serverTask;
	
	// define input-ports
	// InputPort VideoImageClient
	Smart::IPushClientPattern<DomainVision::CommVideoImage> *videoImageClient;
	Smart::InputTaskTrigger<DomainVision::CommVideoImage> *videoImageClientInputTaskTrigger;
	VideoImageClientUpcallManager *videoImageClientUpcallManager;
	VideoImageClientInputCollector *videoImageClientInputCollector;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	
	// define answer-ports
	
	// define request-handlers
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	StateActivityManager *stateActivityManager;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentRGBToWSStreamPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentRGBToWSStreamExtension *extension);
	
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
	
	Smart::StatusCode connectVideoImageClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentRGBToWSStream* instance()
	{
		if(_componentRGBToWSStream == 0) {
			_componentRGBToWSStream = new ComponentRGBToWSStream();
		}
		return _componentRGBToWSStream;
	}
	
	static void deleteInstance() {
		if(_componentRGBToWSStream != 0) {
			delete _componentRGBToWSStream;
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
		struct CameraTask_struct {
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
		} cameraTask;
		struct ServerTask_struct {
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
		} serverTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
	
		//--- client port parameter ---
		struct VideoImageClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} videoImageClient;
		
	} connections;
};
#endif
