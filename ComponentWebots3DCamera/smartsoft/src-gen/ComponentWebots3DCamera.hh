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
#ifndef _COMPONENTWEBOTS3DCAMERA_HH
#define _COMPONENTWEBOTS3DCAMERA_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentWebots3DCameraCore.hh"

#include "ComponentWebots3DCameraPortFactoryInterface.hh"
#include "ComponentWebots3DCameraExtension.hh"

// forward declarations
class ComponentWebots3DCameraPortFactoryInterface;
class ComponentWebots3DCameraExtension;

// includes for PlainOpcUaComponentWebots3DCameraExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBaseStateACE.hh>
#include <DomainVision/CommDepthImage.hh>
#include <DomainVision/CommDepthImageACE.hh>
#include <CommBasicObjects/CommDevicePoseState.hh>
#include <CommBasicObjects/CommDevicePoseStateACE.hh>
#include <CommManipulatorObjects/CommMobileManipulatorState.hh>
#include <CommManipulatorObjects/CommMobileManipulatorStateACE.hh>
#include <DomainVision/CommRGBDImage.hh>
#include <DomainVision/CommRGBDImageACE.hh>
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommVoidACE.hh>

// include tasks
#include "ImageTask.hh"
// include UpcallManagers and InputCollectors
#include "UrPosePushTimedClientUpcallManager.hh"
#include "UrPosePushTimedClientInputCollector.hh"
#include "BasePushTimedClientUpcallManager.hh"
#include "BasePushTimedClientInputCollector.hh"
#include "PtuPosePushNewestClientUpcallManager.hh"
#include "PtuPosePushNewestClientInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
#include "ColorImageQueryHandler.hh"
#include "ImageQueryHandler.hh"
// output port wrappers
#include "RGBImagePushServiceOutWrapper.hh"
#include "DepthPushNewestServerWrapper.hh"
#include "RGBDImagePushServiceOutWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"

#define COMP ComponentWebots3DCamera::instance()

class ComponentWebots3DCamera : public ComponentWebots3DCameraCore {
private:
	static ComponentWebots3DCamera *_componentWebots3DCamera;
	
	// constructor
	ComponentWebots3DCamera();
	
	// copy-constructor
	ComponentWebots3DCamera(const ComponentWebots3DCamera& cc);
	
	// destructor
	~ComponentWebots3DCamera() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentWebots3DCameraPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentWebots3DCameraExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* imageTaskTrigger;
	ImageTask *imageTask;
	
	// define input-ports
	// InputPort UrPosePushTimedClient
	Smart::IPushClientPattern<CommManipulatorObjects::CommMobileManipulatorState> *urPosePushTimedClient;
	Smart::InputTaskTrigger<CommManipulatorObjects::CommMobileManipulatorState> *urPosePushTimedClientInputTaskTrigger;
	UrPosePushTimedClientUpcallManager *urPosePushTimedClientUpcallManager;
	UrPosePushTimedClientInputCollector *urPosePushTimedClientInputCollector;
	// InputPort basePushTimedClient
	Smart::IPushClientPattern<CommBasicObjects::CommBaseState> *basePushTimedClient;
	Smart::InputTaskTrigger<CommBasicObjects::CommBaseState> *basePushTimedClientInputTaskTrigger;
	BasePushTimedClientUpcallManager *basePushTimedClientUpcallManager;
	BasePushTimedClientInputCollector *basePushTimedClientInputCollector;
	// InputPort ptuPosePushNewestClient
	Smart::IPushClientPattern<CommBasicObjects::CommDevicePoseState> *ptuPosePushNewestClient;
	Smart::InputTaskTrigger<CommBasicObjects::CommDevicePoseState> *ptuPosePushNewestClientInputTaskTrigger;
	PtuPosePushNewestClientUpcallManager *ptuPosePushNewestClientUpcallManager;
	PtuPosePushNewestClientInputCollector *ptuPosePushNewestClientInputCollector;
	
	// define request-ports
	Smart::IQueryClientPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState> *urPoseQueryClient;
	
	// define input-handler
	
	// define output-ports
	Smart::IPushServerPattern<DomainVision::CommRGBDImage> *rGBDImagePushServiceOut;
	RGBDImagePushServiceOutWrapper *rGBDImagePushServiceOutWrapper;
	Smart::IPushServerPattern<DomainVision::CommVideoImage> *rGBImagePushServiceOut;
	RGBImagePushServiceOutWrapper *rGBImagePushServiceOutWrapper;
	Smart::IPushServerPattern<DomainVision::CommDepthImage> *depthPushNewestServer;
	DepthPushNewestServerWrapper *depthPushNewestServerWrapper;
	
	// define answer-ports
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommVideoImage> *colorImageQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, DomainVision::CommVideoImage> *colorImageQueryServerInputTaskTrigger;
	Smart::IQueryServerPattern<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> *imageQueryServer;
	Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, DomainVision::CommRGBDImage> *imageQueryServerInputTaskTrigger;
	
	// define request-handlers
	ColorImageQueryHandler *colorImageQueryHandler;
	ImageQueryHandler *imageQueryHandler;
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentWebots3DCameraPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentWebots3DCameraExtension *extension);
	
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
	
	Smart::StatusCode connectUrPosePushTimedClient(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectUrPoseQueryClient(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectBasePushTimedClient(const std::string &serverName, const std::string &serviceName);
	Smart::StatusCode connectPtuPosePushNewestClient(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentWebots3DCamera* instance()
	{
		if(_componentWebots3DCamera == 0) {
			_componentWebots3DCamera = new ComponentWebots3DCamera();
		}
		return _componentWebots3DCamera;
	}
	
	static void deleteInstance() {
		if(_componentWebots3DCamera != 0) {
			delete _componentWebots3DCamera;
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
		struct ImageTask_struct {
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
		} imageTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct RGBDImagePushServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} rGBDImagePushServiceOut;
		struct RGBImagePushServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} rGBImagePushServiceOut;
		struct ColorImageQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} colorImageQueryServer;
		struct DepthPushNewestServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} depthPushNewestServer;
		struct ImageQueryServer_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} imageQueryServer;
	
		//--- client port parameter ---
		struct UrPosePushTimedClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} urPosePushTimedClient;
		struct UrPoseQueryClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} urPoseQueryClient;
		struct BasePushTimedClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} basePushTimedClient;
		struct PtuPosePushNewestClient_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} ptuPosePushNewestClient;
		
	} connections;
};
#endif
