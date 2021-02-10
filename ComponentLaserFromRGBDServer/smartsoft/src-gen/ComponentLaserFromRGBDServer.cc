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
#include "ComponentLaserFromRGBDServer.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentLaserFromRGBDServerAcePortFactory.hh"


// initialize static singleton pointer to zero
ComponentLaserFromRGBDServer* ComponentLaserFromRGBDServer::_componentLaserFromRGBDServer = 0;

// constructor
ComponentLaserFromRGBDServer::ComponentLaserFromRGBDServer()
{
	std::cout << "constructor of ComponentLaserFromRGBDServer\n";
	
	// set all pointer members to NULL
	//coordinationPort = NULL;
	//coordinationPort = NULL;
	laserQueryHandler = NULL;
	laserServiceOut = NULL;
	laserServiceOutWrapper = NULL;
	laserTask = NULL;
	laserTaskTrigger = NULL;
	visTask = NULL;
	visTaskTrigger = NULL;
	laserQueryServer = NULL;
	laserQueryServerInputTaskTrigger = NULL;
	rgbdClient = NULL;
	rgbdClientInputTaskTrigger = NULL;
	rgbdClientUpcallManager = NULL;
	rgbdClientInputCollector = NULL;
	stateChangeHandler = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentLaserFromRGBDServer";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.laserServiceOut.serviceName = "LaserServiceOut";
	connections.laserServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.laserQueryServer.serviceName = "laserQueryServer";
	connections.laserQueryServer.roboticMiddleware = "ACE_SmartSoft";
	connections.rgbdClient.initialConnect = false;
	connections.rgbdClient.wiringName = "rgbdClient";
	connections.rgbdClient.serverName = "unknown";
	connections.rgbdClient.serviceName = "unknown";
	connections.rgbdClient.interval = 1;
	connections.rgbdClient.roboticMiddleware = "ACE_SmartSoft";
	connections.laserTask.minActFreq = 0.0;
	connections.laserTask.maxActFreq = 0.0;
	connections.laserTask.trigger = "PeriodicTimer";
	connections.laserTask.periodicActFreq = 5.0;
	// scheduling default parameters
	connections.laserTask.scheduler = "DEFAULT";
	connections.laserTask.priority = -1;
	connections.laserTask.cpuAffinity = -1;
	connections.visTask.minActFreq = 0.0;
	connections.visTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.visTask.scheduler = "DEFAULT";
	connections.visTask.priority = -1;
	connections.visTask.cpuAffinity = -1;
	
	// initialize members of ComponentLaserFromRGBDServerROS1InterfacesExtension
	
	// initialize members of ComponentLaserFromRGBDServerROSExtension
	
	// initialize members of ComponentLaserFromRGBDServerRestInterfacesExtension
	
	// initialize members of OpcUaBackendComponentGeneratorExtension
	
	// initialize members of PlainOpcUaComponentLaserFromRGBDServerExtension
	
}

void ComponentLaserFromRGBDServer::addPortFactory(const std::string &name, ComponentLaserFromRGBDServerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentLaserFromRGBDServer::addExtension(ComponentLaserFromRGBDServerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentLaserFromRGBDServer::getComponentImpl()
{
	return dynamic_cast<ComponentLaserFromRGBDServerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentLaserFromRGBDServer::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentLaserFromRGBDServer::connectRgbdClient(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.rgbdClient.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = rgbdClient->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->rgbdClient->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	rgbdClient->subscribe(connections.rgbdClient.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentLaserFromRGBDServer::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectRgbdClient(connections.rgbdClient.serverName, connections.rgbdClient.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentLaserFromRGBDServer::startAllTasks() {
	// start task LaserTask
	if(connections.laserTask.scheduler != "DEFAULT") {
		ACE_Sched_Params laserTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.laserTask.scheduler == "FIFO") {
			laserTask_SchedParams.policy(ACE_SCHED_FIFO);
			laserTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.laserTask.scheduler == "RR") {
			laserTask_SchedParams.policy(ACE_SCHED_RR);
			laserTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		laserTask->start(laserTask_SchedParams, connections.laserTask.cpuAffinity);
	} else {
		laserTask->start();
	}
	// start task VisTask
	if(connections.visTask.scheduler != "DEFAULT") {
		ACE_Sched_Params visTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.visTask.scheduler == "FIFO") {
			visTask_SchedParams.policy(ACE_SCHED_FIFO);
			visTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
		} else if(connections.visTask.scheduler == "RR") {
			visTask_SchedParams.policy(ACE_SCHED_RR);
			visTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
		}
		visTask->start(visTask_SchedParams, connections.visTask.cpuAffinity);
	} else {
		visTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentLaserFromRGBDServer::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentLaserFromRGBDServer::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "rgbdClient") return rgbdClientInputTaskTrigger;
	
	return NULL;
}


void ComponentLaserFromRGBDServer::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		// initializations of ComponentLaserFromRGBDServerROS1InterfacesExtension
		
		// initializations of ComponentLaserFromRGBDServerROSExtension
		
		// initializations of ComponentLaserFromRGBDServerRestInterfacesExtension
		
		// initializations of OpcUaBackendComponentGeneratorExtension
		
		// initializations of PlainOpcUaComponentLaserFromRGBDServerExtension
		
		
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
		
		ComponentLaserFromRGBDServerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentLaserFromRGBDServerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentLaserFromRGBDServer is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		laserServiceOut = portFactoryRegistry[connections.laserServiceOut.roboticMiddleware]->createLaserServiceOut(connections.laserServiceOut.serviceName);
		laserServiceOutWrapper = new LaserServiceOutWrapper(laserServiceOut);
		laserQueryServer = portFactoryRegistry[connections.laserQueryServer.roboticMiddleware]->createLaserQueryServer(connections.laserQueryServer.serviceName);
		laserQueryServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan>(laserQueryServer);
		
		// create client ports
		rgbdClient = portFactoryRegistry[connections.rgbdClient.roboticMiddleware]->createRgbdClient();
		
		// create InputTaskTriggers and UpcallManagers
		rgbdClientInputCollector = new RgbdClientInputCollector(rgbdClient);
		rgbdClientInputTaskTrigger = new Smart::InputTaskTrigger<DomainVision::CommRGBDImage>(rgbdClientInputCollector);
		rgbdClientUpcallManager = new RgbdClientUpcallManager(rgbdClientInputCollector);
		
		// create input-handler
		
		// create request-handlers
		laserQueryHandler = new LaserQueryHandler(laserQueryServer);
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateSlave = new SmartACE::StateSlave(component, stateChangeHandler);
		if (stateSlave->defineStates("GenerateLaser" ,"GenerateLaser") != Smart::SMART_OK) std::cerr << "ERROR: defining state combinaion GenerateLaser.GenerateLaser" << std::endl;
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.rgbdClient.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<DomainVision::CommRGBDImage>*>(rgbdClient)->add(wiringSlave, connections.rgbdClient.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task LaserTask
		laserTask = new LaserTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.laserTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.laserTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(laserTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				laserTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task LaserTask" << std::endl;
			}
		} else if(connections.laserTask.trigger == "DataTriggered") {
			laserTaskTrigger = getInputTaskTriggerFromString(connections.laserTask.inPortRef);
			if(laserTaskTrigger != NULL) {
				laserTaskTrigger->attach(laserTask, connections.laserTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.laserTask.inPortRef << " as activation source for Task LaserTask" << std::endl;
			}
		} else
		{
			// setup default task-trigger as PeriodicTimer
			Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
			int microseconds = 1000*1000 / 5.0;
			if(microseconds > 0) {
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				triggerPtr->attach(laserTask);
				// store trigger in class member
				laserTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task LaserTask" << std::endl;
			}
		}
		
		// create Task VisTask
		visTask = new VisTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.visTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = 1000*1000 / connections.visTask.periodicActFreq;
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(visTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				visTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task VisTask" << std::endl;
			}
		} else if(connections.visTask.trigger == "DataTriggered") {
			visTaskTrigger = getInputTaskTriggerFromString(connections.visTask.inPortRef);
			if(visTaskTrigger != NULL) {
				visTaskTrigger->attach(visTask, connections.visTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.visTask.inPortRef << " as activation source for Task VisTask" << std::endl;
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
void ComponentLaserFromRGBDServer::run()
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
void ComponentLaserFromRGBDServer::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(laserTaskTrigger != NULL){
		laserTaskTrigger->detach(laserTask);
		delete laserTask;
	}
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(visTaskTrigger != NULL){
		visTaskTrigger->detach(visTask);
		delete visTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete rgbdClientInputTaskTrigger;
	delete rgbdClientUpcallManager;
	delete rgbdClientInputCollector;

	// destroy client ports
	delete rgbdClient;

	// destroy server ports
	delete laserServiceOutWrapper;
	delete laserServiceOut;
	delete laserQueryServer;
	delete laserQueryServerInputTaskTrigger;
	// destroy event-test handlers (if needed)
	
	// destroy request-handlers
	delete laserQueryHandler;
	
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
	
	// destruction of ComponentLaserFromRGBDServerROS1InterfacesExtension
	
	// destruction of ComponentLaserFromRGBDServerROSExtension
	
	// destruction of ComponentLaserFromRGBDServerRestInterfacesExtension
	
	// destruction of OpcUaBackendComponentGeneratorExtension
	
	// destruction of PlainOpcUaComponentLaserFromRGBDServerExtension
	
}

void ComponentLaserFromRGBDServer::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentLaserFromRGBDServer.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentLaserFromRGBDServer.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentLaserFromRGBDServer.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client rgbdClient
		parameter.getBoolean("rgbdClient", "initialConnect", connections.rgbdClient.initialConnect);
		parameter.getString("rgbdClient", "serviceName", connections.rgbdClient.serviceName);
		parameter.getString("rgbdClient", "serverName", connections.rgbdClient.serverName);
		parameter.getString("rgbdClient", "wiringName", connections.rgbdClient.wiringName);
		parameter.getInteger("rgbdClient", "interval", connections.rgbdClient.interval);
		if(parameter.checkIfParameterExists("rgbdClient", "roboticMiddleware")) {
			parameter.getString("rgbdClient", "roboticMiddleware", connections.rgbdClient.roboticMiddleware);
		}
		
		// load parameters for server LaserServiceOut
		parameter.getString("LaserServiceOut", "serviceName", connections.laserServiceOut.serviceName);
		if(parameter.checkIfParameterExists("LaserServiceOut", "roboticMiddleware")) {
			parameter.getString("LaserServiceOut", "roboticMiddleware", connections.laserServiceOut.roboticMiddleware);
		}
		// load parameters for server laserQueryServer
		parameter.getString("laserQueryServer", "serviceName", connections.laserQueryServer.serviceName);
		if(parameter.checkIfParameterExists("laserQueryServer", "roboticMiddleware")) {
			parameter.getString("laserQueryServer", "roboticMiddleware", connections.laserQueryServer.roboticMiddleware);
		}
		
		// load parameters for task LaserTask
		parameter.getDouble("LaserTask", "minActFreqHz", connections.laserTask.minActFreq);
		parameter.getDouble("LaserTask", "maxActFreqHz", connections.laserTask.maxActFreq);
		parameter.getString("LaserTask", "triggerType", connections.laserTask.trigger);
		if(connections.laserTask.trigger == "PeriodicTimer") {
			parameter.getDouble("LaserTask", "periodicActFreqHz", connections.laserTask.periodicActFreq);
		} else if(connections.laserTask.trigger == "DataTriggered") {
			parameter.getString("LaserTask", "inPortRef", connections.laserTask.inPortRef);
			parameter.getInteger("LaserTask", "prescale", connections.laserTask.prescale);
		}
		if(parameter.checkIfParameterExists("LaserTask", "scheduler")) {
			parameter.getString("LaserTask", "scheduler", connections.laserTask.scheduler);
		}
		if(parameter.checkIfParameterExists("LaserTask", "priority")) {
			parameter.getInteger("LaserTask", "priority", connections.laserTask.priority);
		}
		if(parameter.checkIfParameterExists("LaserTask", "cpuAffinity")) {
			parameter.getInteger("LaserTask", "cpuAffinity", connections.laserTask.cpuAffinity);
		}
		// load parameters for task VisTask
		parameter.getDouble("VisTask", "minActFreqHz", connections.visTask.minActFreq);
		parameter.getDouble("VisTask", "maxActFreqHz", connections.visTask.maxActFreq);
		parameter.getString("VisTask", "triggerType", connections.visTask.trigger);
		if(connections.visTask.trigger == "PeriodicTimer") {
			parameter.getDouble("VisTask", "periodicActFreqHz", connections.visTask.periodicActFreq);
		} else if(connections.visTask.trigger == "DataTriggered") {
			parameter.getString("VisTask", "inPortRef", connections.visTask.inPortRef);
			parameter.getInteger("VisTask", "prescale", connections.visTask.prescale);
		}
		if(parameter.checkIfParameterExists("VisTask", "scheduler")) {
			parameter.getString("VisTask", "scheduler", connections.visTask.scheduler);
		}
		if(parameter.checkIfParameterExists("VisTask", "priority")) {
			parameter.getInteger("VisTask", "priority", connections.visTask.priority);
		}
		if(parameter.checkIfParameterExists("VisTask", "cpuAffinity")) {
			parameter.getInteger("VisTask", "cpuAffinity", connections.visTask.cpuAffinity);
		}
		
		// load parameters for ComponentLaserFromRGBDServerROS1InterfacesExtension
		
		// load parameters for ComponentLaserFromRGBDServerROSExtension
		
		// load parameters for ComponentLaserFromRGBDServerRestInterfacesExtension
		
		// load parameters for OpcUaBackendComponentGeneratorExtension
		
		// load parameters for PlainOpcUaComponentLaserFromRGBDServerExtension
		
		
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
