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
#ifndef _COMPONENTAPRILTAGDETECTOR_HH
#define _COMPONENTAPRILTAGDETECTOR_HH

#include <map>
#include <iostream>
#include "aceSmartSoft.hh"
#include "smartQueryServerTaskTrigger_T.h"
#include "ComponentAprilTagDetectorCore.hh"

#include "ComponentAprilTagDetectorPortFactoryInterface.hh"
#include "ComponentAprilTagDetectorExtension.hh"

// forward declarations
class ComponentAprilTagDetectorPortFactoryInterface;
class ComponentAprilTagDetectorExtension;

// includes for PlainOpcUaComponentAprilTagDetectorExtension
// include plain OPC UA device clients
// include plain OPC UA status servers


// include communication objects
#include <CommTrackingObjects/CommDetectedMarkerEventParameter.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventParameterACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventResult.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventResultACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventState.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventStateACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerList.hh>
#include <CommTrackingObjects/CommDetectedMarkerListACE.hh>
#include <DomainVision/CommVideoImage.hh>
#include <DomainVision/CommVideoImageACE.hh>

// include tasks
#include "DetectorTask.hh"
// include UpcallManagers and InputCollectors
#include "RGBImagePushServiceInUpcallManager.hh"
#include "RGBImagePushServiceInInputCollector.hh"

// include input-handler(s)
// include request-handler(s)
// output port wrappers
#include "RGBImagePushServiceOutWrapper.hh"
#include "MarkerListDetectionServiceOutWrapper.hh"
#include "MarkerListEventServiceOutWrapper.hh"

// include handler
#include "CompHandler.hh"

#include "ParameterStateStruct.hh"
#include "ParameterUpdateHandler.hh"

#include "SmartStateChangeHandler.hh"
#include "StateActivityManager.hh"


#define COMP ComponentAprilTagDetector::instance()

class ComponentAprilTagDetector : public ComponentAprilTagDetectorCore {
private:
	static ComponentAprilTagDetector *_componentAprilTagDetector;
	
	// constructor
	ComponentAprilTagDetector();
	
	// copy-constructor
	ComponentAprilTagDetector(const ComponentAprilTagDetector& cc);
	
	// destructor
	~ComponentAprilTagDetector() { };
	
	// load parameter from ini file
	void loadParameter(int argc, char* argv[]);
	
	// instantiate comp-handler
	CompHandler compHandler;
	
	// helper method that maps a string-name to an according TaskTriggerSubject
	Smart::TaskTriggerSubject* getInputTaskTriggerFromString(const std::string &client);
	
	// internal map storing the different port-creation factories (that internally map to specific middleware implementations)
	std::map<std::string, ComponentAprilTagDetectorPortFactoryInterface*> portFactoryRegistry;
	
	// internal map storing various extensions of this component class
	std::map<std::string, ComponentAprilTagDetectorExtension*> componentExtensionRegistry;
	
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
	Smart::TaskTriggerSubject* detectorTaskTrigger;
	DetectorTask *detectorTask;
	
	// define input-ports
	// InputPort RGBImagePushServiceIn
	Smart::IPushClientPattern<DomainVision::CommVideoImage> *rGBImagePushServiceIn;
	Smart::InputTaskTrigger<DomainVision::CommVideoImage> *rGBImagePushServiceInInputTaskTrigger;
	RGBImagePushServiceInUpcallManager *rGBImagePushServiceInUpcallManager;
	RGBImagePushServiceInInputCollector *rGBImagePushServiceInInputCollector;
	
	// define request-ports
	
	// define input-handler
	
	// define output-ports
	Smart::IPushServerPattern<CommTrackingObjects::CommDetectedMarkerList> *markerListDetectionServiceOut;
	MarkerListDetectionServiceOutWrapper *markerListDetectionServiceOutWrapper;
	Smart::IEventServerPattern<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState> *markerListEventServiceOut;
	MarkerListEventServiceOutWrapper *markerListEventServiceOutWrapper;
	std::shared_ptr<Smart::IEventTestHandler<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState>> markerListEventServiceOutEventTestHandler;
	Smart::IPushServerPattern<DomainVision::CommVideoImage> *rGBImagePushServiceOut;
	RGBImagePushServiceOutWrapper *rGBImagePushServiceOutWrapper;
	
	// define answer-ports
	
	// define request-handlers
	
	
	// define default slave ports
	SmartACE::StateSlave *stateSlave;
	StateActivityManager *stateActivityManager;
	SmartStateChangeHandler *stateChangeHandler;
	SmartACE::WiringSlave *wiringSlave;
	ParamUpdateHandler paramHandler;
	SmartACE::ParameterSlave *param;
	
	
	/// this method is used to register different PortFactory classes (one for each supported middleware framework)
	void addPortFactory(const std::string &name, ComponentAprilTagDetectorPortFactoryInterface *portFactory);
	
	SmartACE::SmartComponent* getComponentImpl();
	
	/// this method is used to register different component-extension classes
	void addExtension(ComponentAprilTagDetectorExtension *extension);
	
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
	
	Smart::StatusCode connectRGBImagePushServiceIn(const std::string &serverName, const std::string &serviceName);

	// return singleton instance
	static ComponentAprilTagDetector* instance()
	{
		if(_componentAprilTagDetector == 0) {
			_componentAprilTagDetector = new ComponentAprilTagDetector();
		}
		return _componentAprilTagDetector;
	}
	
	static void deleteInstance() {
		if(_componentAprilTagDetector != 0) {
			delete _componentAprilTagDetector;
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
		struct DetectorTask_struct {
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
		} detectorTask;
		
		//--- upcall parameter ---
		
		//--- server port parameter ---
		struct MarkerListDetectionServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} markerListDetectionServiceOut;
		struct MarkerListEventServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} markerListEventServiceOut;
		struct RGBImagePushServiceOut_struct {
				std::string serviceName;
				std::string roboticMiddleware;
		} rGBImagePushServiceOut;
	
		//--- client port parameter ---
		struct RGBImagePushServiceIn_struct {
			bool initialConnect;
			std::string serverName;
			std::string serviceName;
			std::string wiringName;
			long interval;
			std::string roboticMiddleware;
		} rGBImagePushServiceIn;
		
	} connections;
};
#endif
