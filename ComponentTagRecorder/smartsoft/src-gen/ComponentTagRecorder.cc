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
#include "ComponentTagRecorder.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentTagRecorderAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentTagRecorder* ComponentTagRecorder::_componentTagRecorder = 0;

// constructor
ComponentTagRecorder::ComponentTagRecorder()
{
	std::cout << "constructor of ComponentTagRecorder\n";
	
	// set all pointer members to NULL
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	markerListDetectionServiceIn = NULL;
	markerListDetectionServiceInInputTaskTrigger = NULL;
	markerListDetectionServiceInUpcallManager = NULL;
	markerListDetectionServiceInInputCollector = NULL;
	recorderTask = NULL;
	recorderTaskTrigger = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentTagRecorder";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.markerListDetectionServiceIn.initialConnect = false;
	connections.markerListDetectionServiceIn.wiringName = "MarkerListDetectionServiceIn";
	connections.markerListDetectionServiceIn.serverName = "unknown";
	connections.markerListDetectionServiceIn.serviceName = "unknown";
	connections.markerListDetectionServiceIn.interval = 1;
	connections.markerListDetectionServiceIn.roboticMiddleware = "ACE_SmartSoft";
	connections.recorderTask.minActFreq = 0.0;
	connections.recorderTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.recorderTask.scheduler = "DEFAULT";
	connections.recorderTask.priority = -1;
	connections.recorderTask.cpuAffinity = -1;
	
	// initialize members of ComponentTagRecorderROS1InterfacesExtension
	
	// initialize members of ComponentTagRecorderROSExtension
	
	// initialize members of ComponentTagRecorderRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentTagRecorderExtension
	
}

void ComponentTagRecorder::addPortFactory(const std::string &name, ComponentTagRecorderPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentTagRecorder::addExtension(ComponentTagRecorderExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentTagRecorder::getComponentImpl()
{
	return dynamic_cast<ComponentTagRecorderAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentTagRecorder::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentTagRecorder::connectMarkerListDetectionServiceIn(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.markerListDetectionServiceIn.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = markerListDetectionServiceIn->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->markerListDetectionServiceIn->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	markerListDetectionServiceIn->subscribe(connections.markerListDetectionServiceIn.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentTagRecorder::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectMarkerListDetectionServiceIn(connections.markerListDetectionServiceIn.serverName, connections.markerListDetectionServiceIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentTagRecorder::startAllTasks() {
	// start task RecorderTask
	if(connections.recorderTask.scheduler != "DEFAULT") {
		ACE_Sched_Params recorderTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.recorderTask.scheduler == "FIFO") {
			recorderTask_SchedParams.policy(ACE_SCHED_FIFO);
			recorderTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.recorderTask.scheduler == "RR") {
			recorderTask_SchedParams.policy(ACE_SCHED_RR);
			recorderTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		recorderTask->start(recorderTask_SchedParams, connections.recorderTask.cpuAffinity);
	} else {
		recorderTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentTagRecorder::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentTagRecorder::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "MarkerListDetectionServiceIn") return markerListDetectionServiceInInputTaskTrigger;
	
	return NULL;
}


void ComponentTagRecorder::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentTagRecorderROS1InterfacesExtension
		
		// initializations of ComponentTagRecorderROSExtension
		
		// initializations of ComponentTagRecorderRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentTagRecorderExtension
		
		
		// initialize all registered port-factories
		for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
		{
			portFactory->second->initialize(this, argc, argv);
		}
		
		// initialize all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->initialize(this, argc, argv);
		}
		
		ComponentTagRecorderPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentTagRecorderAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentTagRecorder is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		
		// create client ports
		markerListDetectionServiceIn = portFactoryRegistry[connections.markerListDetectionServiceIn.roboticMiddleware]->createMarkerListDetectionServiceIn();
		
		// create InputTaskTriggers and UpcallManagers
		markerListDetectionServiceInInputCollector = new MarkerListDetectionServiceInInputCollector(markerListDetectionServiceIn);
		markerListDetectionServiceInInputTaskTrigger = new Smart::InputTaskTrigger<CommTrackingObjects::CommDetectedMarkerList>(markerListDetectionServiceInInputCollector);
		markerListDetectionServiceInUpcallManager = new MarkerListDetectionServiceInUpcallManager(markerListDetectionServiceInInputCollector);
		
		// create input-handler
		
		// create request-handlers
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		if (stateSlave->defineStates("Active" ,"active") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion Active.active" << std::endl;
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.markerListDetectionServiceIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommTrackingObjects::CommDetectedMarkerList>*>(markerListDetectionServiceIn)->add(wiringSlave, connections.markerListDetectionServiceIn.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task RecorderTask
		recorderTask = new RecorderTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.recorderTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.recorderTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(recorderTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				recorderTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task RecorderTask" << std::endl;
			}
		} else if(connections.recorderTask.trigger == "DataTriggered") {
			recorderTaskTrigger = getInputTaskTriggerFromString(connections.recorderTask.inPortRef);
			if(recorderTaskTrigger != NULL) {
				recorderTaskTrigger->attach(recorderTask, connections.recorderTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.recorderTask.inPortRef << " as activation source for Task RecorderTask" << std::endl;
			}
		} 
		
		
		// link observers with subjects
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void ComponentTagRecorder::run()
{
	stateSlave->acquire("init");
	// startup all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onStartup();
	}
	
	// startup all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onStartup();
	}
	stateSlave->release("init");
	
	// do not call this handler within the init state (see above) as this handler internally calls setStartupFinished() (this should be fixed in future)
	compHandler.onStartup();
	
	// this call blocks until the component is commanded to shutdown
	stateSlave->acquire("shutdown");
	
	// shutdown all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->onShutdown();
	}
	
	// shutdown all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->onShutdown();
	}
	
	if(connections.component.useLogger == true) {
		//FIXME: use logging
		//Smart::LOGGER->stopLogging();
	}
	
	compHandler.onShutdown();
	
	stateSlave->release("shutdown");
}

// clean-up component's resources
void ComponentTagRecorder::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(recorderTaskTrigger != NULL){
		recorderTaskTrigger->detach(recorderTask);
		delete recorderTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete markerListDetectionServiceInInputTaskTrigger;
	delete markerListDetectionServiceInUpcallManager;
	delete markerListDetectionServiceInInputCollector;

	// destroy client ports
	delete markerListDetectionServiceIn;

	// destroy server ports
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	
	delete stateSlave;
	// destroy state-change-handler
	delete stateChangeHandler;
	
	// destroy all master/slave ports
	delete wiringSlave;
	delete param;
	

	// destroy all registered component-extensions
	for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
	{
		extension->second->destroy();
	}

	// destroy all registered port-factories
	for(auto portFactory = portFactoryRegistry.begin(); portFactory != portFactoryRegistry.end(); portFactory++) 
	{
		portFactory->second->destroy();
	}
	
	// destruction of ComponentTagRecorderROS1InterfacesExtension
	
	// destruction of ComponentTagRecorderROSExtension
	
	// destruction of ComponentTagRecorderRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentTagRecorderExtension
	
}

void ComponentTagRecorder::loadParameter(int argc, char *argv[])
{
	/*
	 Parameters can be specified via command line --filename=<filename> or -f <filename>

	 With this parameter present:
	   - The component will look for the file in the current working directory,
	     a path relative to the current directory or any absolute path
	   - The component will use the default values if the file cannot be found

	 With this parameter absent:
	   - <Name of Component>.ini will be read from current working directory, if found there
	   - $SMART_ROOT/etc/<Name of Component>.ini will be read otherwise
	   - Default values will be used if neither found in working directory or /etc
	 */
	SmartACE::SmartIniParameter parameter;
	std::ifstream parameterfile;
	bool parameterFileFound = false;

	// load parameters
	try
	{
		// if paramfile is given as argument
		if(parameter.tryAddFileFromArgs(argc,argv,"filename", 'f'))
		{
			parameterFileFound = true;
			std::cout << "parameter file is loaded from an argv argument \n";
		} else if(parameter.searchFile("ComponentTagRecorder.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentTagRecorder.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentTagRecorder.ini parameter file not found! (using default values or command line arguments)\n";
		}
		
		// add command line arguments to allow overwriting of parameters
		// from file
		parameter.addCommandLineArgs(argc,argv,"component");
		
		// initialize the naming service using the command line parameters parsed in the
		// SmartIniParameter class. The naming service parameters are expected to be in
		// the "component" parameter group.
		SmartACE::NAMING::instance()->checkForHelpArg(argc,argv);
		if(parameterFileFound) 
		{
			if(SmartACE::NAMING::instance()->init(parameter.getAllParametersFromGroup("component")) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		} else {
			if(SmartACE::NAMING::instance()->init(argc, argv) != 0) {
				// initialization of naming service failed
				throw std::logic_error( "<NamingService> Service initialization failed!\nPossible causes could be:\n-> Erroneous configuration.\n-> Naming service not reachable.\n" );
			}
		}
			
		// print all known parameters
		// parameter.print();
		
		//--- server port // client port // other parameter ---
		// load parameter
		parameter.getString("component", "name", connections.component.name);
		parameter.getString("component", "initialComponentMode", connections.component.initialComponentMode);
		if(parameter.checkIfParameterExists("component", "defaultScheduler")) {
			parameter.getString("component", "defaultScheduler", connections.component.defaultScheduler);
		}
		if(parameter.checkIfParameterExists("component", "useLogger")) {
			parameter.getBoolean("component", "useLogger", connections.component.useLogger);
		}
		
		// load parameters for client MarkerListDetectionServiceIn
		parameter.getBoolean("MarkerListDetectionServiceIn", "initialConnect", connections.markerListDetectionServiceIn.initialConnect);
		parameter.getString("MarkerListDetectionServiceIn", "serviceName", connections.markerListDetectionServiceIn.serviceName);
		parameter.getString("MarkerListDetectionServiceIn", "serverName", connections.markerListDetectionServiceIn.serverName);
		parameter.getString("MarkerListDetectionServiceIn", "wiringName", connections.markerListDetectionServiceIn.wiringName);
		parameter.getInteger("MarkerListDetectionServiceIn", "interval", connections.markerListDetectionServiceIn.interval);
		if(parameter.checkIfParameterExists("MarkerListDetectionServiceIn", "roboticMiddleware")) {
			parameter.getString("MarkerListDetectionServiceIn", "roboticMiddleware", connections.markerListDetectionServiceIn.roboticMiddleware);
		}
		
		
		// load parameters for task RecorderTask
		parameter.getDouble("RecorderTask", "minActFreqHz", connections.recorderTask.minActFreq);
		parameter.getDouble("RecorderTask", "maxActFreqHz", connections.recorderTask.maxActFreq);
		parameter.getString("RecorderTask", "triggerType", connections.recorderTask.trigger);
		if(connections.recorderTask.trigger == "PeriodicTimer") {
			parameter.getDouble("RecorderTask", "periodicActFreqHz", connections.recorderTask.periodicActFreq);
		} else if(connections.recorderTask.trigger == "DataTriggered") {
			parameter.getString("RecorderTask", "inPortRef", connections.recorderTask.inPortRef);
			parameter.getInteger("RecorderTask", "prescale", connections.recorderTask.prescale);
		}
		if(parameter.checkIfParameterExists("RecorderTask", "scheduler")) {
			parameter.getString("RecorderTask", "scheduler", connections.recorderTask.scheduler);
		}
		if(parameter.checkIfParameterExists("RecorderTask", "priority")) {
			parameter.getInteger("RecorderTask", "priority", connections.recorderTask.priority);
		}
		if(parameter.checkIfParameterExists("RecorderTask", "cpuAffinity")) {
			parameter.getInteger("RecorderTask", "cpuAffinity", connections.recorderTask.cpuAffinity);
		}
		
		// load parameters for ComponentTagRecorderROS1InterfacesExtension
		
		// load parameters for ComponentTagRecorderROSExtension
		
		// load parameters for ComponentTagRecorderRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentTagRecorderExtension
		
		
		// load parameters for all registered component-extensions
		for(auto extension = componentExtensionRegistry.begin(); extension != componentExtensionRegistry.end(); extension++) 
		{
			extension->second->loadParameters(parameter);
		}
		
		paramHandler.loadParameter(parameter);
	
	} catch (const SmartACE::IniParameterError & e) {
		std::cerr << e.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std::exception: " << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}
