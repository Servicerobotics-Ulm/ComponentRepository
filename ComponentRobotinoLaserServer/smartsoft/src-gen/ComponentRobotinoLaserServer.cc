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
#include "ComponentRobotinoLaserServer.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "ComponentRobotinoLaserServerAcePortFactory.hh"

#include "SafetyfieldEventServerEventTestHandler.hh"

// initialize static singleton pointer to zero
ComponentRobotinoLaserServer* ComponentRobotinoLaserServer::_componentRobotinoLaserServer = 0;

// constructor
ComponentRobotinoLaserServer::ComponentRobotinoLaserServer()
{
	std::cout << "constructor of ComponentRobotinoLaserServer\n";
	
	// set all pointer members to NULL
	//componentRobotinoLaserServer = NULL;
	//coordinationPort = NULL;
	laserServiceOut = NULL;
	laserServiceOutWrapper = NULL;
	queryHandler = NULL;
	readLaserTask = NULL;
	readLaserTaskTrigger = NULL;
	baseTimedClient = NULL;
	baseTimedClientInputTaskTrigger = NULL;
	baseTimedClientUpcallManager = NULL;
	baseTimedClientInputCollector = NULL;
	queryServer = NULL;
	queryServerInputTaskTrigger = NULL;
	safetyfieldEventServer = NULL;
	safetyfieldEventServerWrapper = NULL;
	safetyfieldEventServerEventTestHandler = nullptr; 
	stateChangeHandler = NULL;
	stateActivityManager = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "ComponentRobotinoLaserServer";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.laserServiceOut.serviceName = "LaserServiceOut";
	connections.laserServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.queryServer.serviceName = "queryServer";
	connections.queryServer.roboticMiddleware = "ACE_SmartSoft";
	connections.safetyfieldEventServer.serviceName = "safetyfieldEventServer";
	connections.safetyfieldEventServer.roboticMiddleware = "ACE_SmartSoft";
	connections.baseTimedClient.initialConnect = false;
	connections.baseTimedClient.wiringName = "baseTimedClient";
	connections.baseTimedClient.serverName = "unknown";
	connections.baseTimedClient.serviceName = "unknown";
	connections.baseTimedClient.interval = 1;
	connections.baseTimedClient.roboticMiddleware = "ACE_SmartSoft";
	connections.readLaserTask.minActFreq = 0.0;
	connections.readLaserTask.maxActFreq = 0.0;
	// scheduling default parameters
	connections.readLaserTask.scheduler = "DEFAULT";
	connections.readLaserTask.priority = -1;
	connections.readLaserTask.cpuAffinity = -1;
	
}

void ComponentRobotinoLaserServer::addPortFactory(const std::string &name, ComponentRobotinoLaserServerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void ComponentRobotinoLaserServer::addExtension(ComponentRobotinoLaserServerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* ComponentRobotinoLaserServer::getComponentImpl()
{
	return dynamic_cast<ComponentRobotinoLaserServerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void ComponentRobotinoLaserServer::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode ComponentRobotinoLaserServer::connectBaseTimedClient(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.baseTimedClient.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = baseTimedClient->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->baseTimedClient->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	baseTimedClient->subscribe(connections.baseTimedClient.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode ComponentRobotinoLaserServer::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBaseTimedClient(connections.baseTimedClient.serverName, connections.baseTimedClient.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void ComponentRobotinoLaserServer::startAllTasks() {
	// start task ReadLaserTask
	if(connections.readLaserTask.scheduler != "DEFAULT") {
		ACE_Sched_Params readLaserTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.readLaserTask.scheduler == "FIFO") {
			readLaserTask_SchedParams.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				readLaserTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				readLaserTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(connections.readLaserTask.scheduler == "RR") {
			readLaserTask_SchedParams.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				readLaserTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				readLaserTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		readLaserTask->start(readLaserTask_SchedParams, connections.readLaserTask.cpuAffinity);
	} else {
		readLaserTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void ComponentRobotinoLaserServer::startAllTimers() {
}


Smart::TaskTriggerSubject* ComponentRobotinoLaserServer::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "baseTimedClient") return baseTimedClientInputTaskTrigger;
	
	return NULL;
}


void ComponentRobotinoLaserServer::init(int argc, char *argv[])
{
	try {
		Smart::StatusCode status;
		
		// load initial parameters from ini-file (if found)
		loadParameter(argc, argv);
		
		// print out the actual parameters which are used to initialize the component
		std::cout << " \nComponentDefinition Initial-Parameters:\n" << COMP->getParameters() << std::endl;
		
		
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
		
		ComponentRobotinoLaserServerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<ComponentRobotinoLaserServerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition ComponentRobotinoLaserServer is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		safetyfieldEventServerEventTestHandler = std::make_shared<SafetyfieldEventServerEventTestHandler>();
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		laserServiceOut = portFactoryRegistry[connections.laserServiceOut.roboticMiddleware]->createLaserServiceOut(connections.laserServiceOut.serviceName);
		laserServiceOutWrapper = new LaserServiceOutWrapper(laserServiceOut);
		queryServer = portFactoryRegistry[connections.queryServer.roboticMiddleware]->createQueryServer(connections.queryServer.serviceName);
		queryServerInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan>(queryServer);
		safetyfieldEventServerEventTestHandler = std::make_shared<SafetyfieldEventServerEventTestHandler>();
		safetyfieldEventServer = portFactoryRegistry[connections.safetyfieldEventServer.roboticMiddleware]->createSafetyfieldEventServer(connections.safetyfieldEventServer.serviceName, safetyfieldEventServerEventTestHandler);
		safetyfieldEventServerWrapper = new SafetyfieldEventServerWrapper(safetyfieldEventServer);
		
		// create client ports
		baseTimedClient = portFactoryRegistry[connections.baseTimedClient.roboticMiddleware]->createBaseTimedClient();
		
		// create InputTaskTriggers and UpcallManagers
		baseTimedClientInputCollector = new BaseTimedClientInputCollector(baseTimedClient);
		baseTimedClientInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(baseTimedClientInputCollector);
		baseTimedClientUpcallManager = new BaseTimedClientUpcallManager(baseTimedClientInputCollector);
		
		// create input-handler
		
		// create request-handlers
		queryHandler = new QueryHandler(queryServer);
		
		// create state pattern
		stateChangeHandler = new SmartStateChangeHandler();
		stateActivityManager = new StateActivityManager(stateChangeHandler);
		stateSlave = new SmartACE::StateSlave(component, stateActivityManager);
		status = stateSlave->setUpInitialState(connections.component.initialComponentMode);
		if (status != Smart::SMART_OK) std::cerr << status << "; failed setting initial ComponentMode: " << connections.component.initialComponentMode << std::endl;
		// activate state slave
		status = stateSlave->activate();
		if(status != Smart::SMART_OK) std::cerr << "ERROR: activate state" << std::endl;
		
		wiringSlave = new SmartACE::WiringSlave(component);
		// add client port to wiring slave
		if(connections.baseTimedClient.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommBaseState>*>(baseTimedClient)->add(wiringSlave, connections.baseTimedClient.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task ReadLaserTask
		readLaserTask = new ReadLaserTask(component);
		// configure input-links
		// configure task-trigger (if task is configurable)
		if(connections.readLaserTask.trigger == "PeriodicTimer") {
			// create PeriodicTimerTrigger
			int microseconds = (int)(1000.0*1000.0 / connections.readLaserTask.periodicActFreq);
			if(microseconds > 0) {
				Smart::TimedTaskTrigger *triggerPtr = new Smart::TimedTaskTrigger();
				triggerPtr->attach(readLaserTask);
				component->getTimerManager()->scheduleTimer(triggerPtr, (void *) 0, std::chrono::microseconds(microseconds), std::chrono::microseconds(microseconds));
				// store trigger in class member
				readLaserTaskTrigger = triggerPtr;
			} else {
				std::cerr << "ERROR: could not set-up Timer with cycle-time " << microseconds << " as activation source for Task ReadLaserTask" << std::endl;
			}
		} else if(connections.readLaserTask.trigger == "DataTriggered") {
			readLaserTaskTrigger = getInputTaskTriggerFromString(connections.readLaserTask.inPortRef);
			if(readLaserTaskTrigger != NULL) {
				readLaserTaskTrigger->attach(readLaserTask, connections.readLaserTask.prescale);
			} else {
				std::cerr << "ERROR: could not set-up InPort " << connections.readLaserTask.inPortRef << " as activation source for Task ReadLaserTask" << std::endl;
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
void ComponentRobotinoLaserServer::run()
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
void ComponentRobotinoLaserServer::fini()
{
	// unlink all observers
	
	// destroy all task instances
	// unlink all UpcallManagers
	// unlink the TaskTrigger
	if(readLaserTaskTrigger != NULL){
		readLaserTaskTrigger->detach(readLaserTask);
		delete readLaserTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete baseTimedClientInputTaskTrigger;
	delete baseTimedClientUpcallManager;
	delete baseTimedClientInputCollector;

	// destroy client ports
	delete baseTimedClient;

	// destroy request-handlers
	delete queryHandler;

	// destroy server ports
	delete laserServiceOutWrapper;
	delete laserServiceOut;
	delete queryServerInputTaskTrigger;
	delete queryServer;
	delete safetyfieldEventServerWrapper;
	delete safetyfieldEventServer;
	
	// destroy event-test handlers (if needed)
	safetyfieldEventServerEventTestHandler;
	
	delete stateSlave;
	delete stateActivityManager;
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
	
}

void ComponentRobotinoLaserServer::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("ComponentRobotinoLaserServer.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load ComponentRobotinoLaserServer.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: ComponentRobotinoLaserServer.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client baseTimedClient
		parameter.getBoolean("baseTimedClient", "initialConnect", connections.baseTimedClient.initialConnect);
		parameter.getString("baseTimedClient", "serviceName", connections.baseTimedClient.serviceName);
		parameter.getString("baseTimedClient", "serverName", connections.baseTimedClient.serverName);
		parameter.getString("baseTimedClient", "wiringName", connections.baseTimedClient.wiringName);
		parameter.getInteger("baseTimedClient", "interval", connections.baseTimedClient.interval);
		if(parameter.checkIfParameterExists("baseTimedClient", "roboticMiddleware")) {
			parameter.getString("baseTimedClient", "roboticMiddleware", connections.baseTimedClient.roboticMiddleware);
		}
		
		// load parameters for server LaserServiceOut
		parameter.getString("LaserServiceOut", "serviceName", connections.laserServiceOut.serviceName);
		if(parameter.checkIfParameterExists("LaserServiceOut", "roboticMiddleware")) {
			parameter.getString("LaserServiceOut", "roboticMiddleware", connections.laserServiceOut.roboticMiddleware);
		}
		// load parameters for server queryServer
		parameter.getString("queryServer", "serviceName", connections.queryServer.serviceName);
		if(parameter.checkIfParameterExists("queryServer", "roboticMiddleware")) {
			parameter.getString("queryServer", "roboticMiddleware", connections.queryServer.roboticMiddleware);
		}
		// load parameters for server safetyfieldEventServer
		parameter.getString("safetyfieldEventServer", "serviceName", connections.safetyfieldEventServer.serviceName);
		if(parameter.checkIfParameterExists("safetyfieldEventServer", "roboticMiddleware")) {
			parameter.getString("safetyfieldEventServer", "roboticMiddleware", connections.safetyfieldEventServer.roboticMiddleware);
		}
		
		// load parameters for task ReadLaserTask
		parameter.getDouble("ReadLaserTask", "minActFreqHz", connections.readLaserTask.minActFreq);
		parameter.getDouble("ReadLaserTask", "maxActFreqHz", connections.readLaserTask.maxActFreq);
		parameter.getString("ReadLaserTask", "triggerType", connections.readLaserTask.trigger);
		if(connections.readLaserTask.trigger == "PeriodicTimer") {
			parameter.getDouble("ReadLaserTask", "periodicActFreqHz", connections.readLaserTask.periodicActFreq);
		} else if(connections.readLaserTask.trigger == "DataTriggered") {
			parameter.getString("ReadLaserTask", "inPortRef", connections.readLaserTask.inPortRef);
			parameter.getInteger("ReadLaserTask", "prescale", connections.readLaserTask.prescale);
		}
		if(parameter.checkIfParameterExists("ReadLaserTask", "scheduler")) {
			parameter.getString("ReadLaserTask", "scheduler", connections.readLaserTask.scheduler);
		}
		if(parameter.checkIfParameterExists("ReadLaserTask", "priority")) {
			parameter.getInteger("ReadLaserTask", "priority", connections.readLaserTask.priority);
		}
		if(parameter.checkIfParameterExists("ReadLaserTask", "cpuAffinity")) {
			parameter.getInteger("ReadLaserTask", "cpuAffinity", connections.readLaserTask.cpuAffinity);
		}
		
		
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
