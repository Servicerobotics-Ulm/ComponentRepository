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
#include "SmartLaserLMS200Server.hh"
#include "smartTimedTaskTrigger.h"
//FIXME: implement logging
//#include "smartGlobalLogger.hh"

// the ace port-factory is used as a default port-mapping
#include "SmartLaserLMS200ServerAcePortFactory.hh"


// initialize static singleton pointer to zero
SmartLaserLMS200Server* SmartLaserLMS200Server::_smartLaserLMS200Server = 0;

// constructor
SmartLaserLMS200Server::SmartLaserLMS200Server()
{
	std::cout << "constructor of SmartLaserLMS200Server\n";
	
	// set all pointer members to NULL
	baseStateIn = NULL;
	baseStateInInputTaskTrigger = NULL;
	baseStateInUpcallManager = NULL;
	baseStateInInputCollector = NULL;
	//coordinationPort = NULL;
	laserQueryServiceAnsw = NULL;
	laserQueryServiceAnswInputTaskTrigger = NULL;
	laserQueryServiceAnswHandler = NULL;
	laserServiceOut = NULL;
	laserServiceOutWrapper = NULL;
	laserTask = NULL;
	laserTaskTrigger = NULL;
	//smartLaserLMS200ServerParams = NULL;
	stateChangeHandler = NULL;
	stateActivityManager = NULL;
	stateSlave = NULL;
	wiringSlave = NULL;
	param = NULL;
	
	// set default ini parameter values
	connections.component.name = "SmartLaserLMS200Server";
	connections.component.initialComponentMode = "Neutral";
	connections.component.defaultScheduler = "DEFAULT";
	connections.component.useLogger = false;
	
	connections.laserQueryServiceAnsw.serviceName = "LaserQueryServiceAnsw";
	connections.laserQueryServiceAnsw.roboticMiddleware = "ACE_SmartSoft";
	connections.laserServiceOut.serviceName = "LaserServiceOut";
	connections.laserServiceOut.roboticMiddleware = "ACE_SmartSoft";
	connections.baseStateIn.initialConnect = false;
	connections.baseStateIn.wiringName = "BaseStateIn";
	connections.baseStateIn.serverName = "unknown";
	connections.baseStateIn.serviceName = "unknown";
	connections.baseStateIn.interval = 1;
	connections.baseStateIn.roboticMiddleware = "ACE_SmartSoft";
	// scheduling default parameters
	connections.laserTask.scheduler = "DEFAULT";
	connections.laserTask.priority = -1;
	connections.laserTask.cpuAffinity = -1;
	
}

void SmartLaserLMS200Server::addPortFactory(const std::string &name, SmartLaserLMS200ServerPortFactoryInterface *portFactory)
{
	portFactoryRegistry[name] = portFactory;
}

void SmartLaserLMS200Server::addExtension(SmartLaserLMS200ServerExtension *extension)
{
	componentExtensionRegistry[extension->getName()] = extension;
}

SmartACE::SmartComponent* SmartLaserLMS200Server::getComponentImpl()
{
	return dynamic_cast<SmartLaserLMS200ServerAcePortFactory*>(portFactoryRegistry["ACE_SmartSoft"])->getComponentImpl();
}

/**
 * Notify the component that setup/initialization is finished.
 * You may call this function from anywhere in the component.
 *
 * Set component's internal lifecycle state automaton (if any) into 
 * Alive mode (from here on the component is ready to provide its services)
 */
void SmartLaserLMS200Server::setStartupFinished() {
	stateSlave->setWaitState("Alive");
	std::cout << "ComponentDefinition initialization/startup finished." << std::endl;
}


Smart::StatusCode SmartLaserLMS200Server::connectBaseStateIn(const std::string &serverName, const std::string &serviceName) {
	Smart::StatusCode status;
	
	if(connections.baseStateIn.initialConnect == false) {
		return Smart::SMART_OK;
	}
	std::cout << "connecting to: " << serverName << "; " << serviceName << std::endl;
	status = baseStateIn->connect(serverName, serviceName);
	while(status != Smart::SMART_OK)
	{
		ACE_OS::sleep(ACE_Time_Value(0,500000));
		status = COMP->baseStateIn->connect(serverName, serviceName);
	}
	std::cout << "connected.\n";
	baseStateIn->subscribe(connections.baseStateIn.interval);
	return status;
}


/**
 * First connect ALL client ports contained in this component, then start all services:
 * activate state, push, etc...
 */
Smart::StatusCode SmartLaserLMS200Server::connectAndStartAllServices() {
	Smart::StatusCode status = Smart::SMART_OK;
	
	status = connectBaseStateIn(connections.baseStateIn.serverName, connections.baseStateIn.serviceName);
	if(status != Smart::SMART_OK) return status;
	return status;
}

/**
 * Start all tasks contained in this component.
 */
void SmartLaserLMS200Server::startAllTasks() {
	// start task LaserTask
	if(connections.laserTask.scheduler != "DEFAULT") {
		ACE_Sched_Params laserTask_SchedParams(ACE_SCHED_OTHER, ACE_THR_PRI_OTHER_DEF);
		if(connections.laserTask.scheduler == "FIFO") {
			laserTask_SchedParams.policy(ACE_SCHED_FIFO);
			#if defined(ACE_HAS_PTHREADS)
				laserTask_SchedParams.priority(ACE_THR_PRI_FIFO_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				laserTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		} else if(connections.laserTask.scheduler == "RR") {
			laserTask_SchedParams.policy(ACE_SCHED_RR);
			#if defined(ACE_HAS_PTHREADS)
				laserTask_SchedParams.priority(ACE_THR_PRI_RR_MIN);
			#elif defined (ACE_HAS_WTHREADS)
				laserTask_SchedParams.priority(THREAD_PRIORITY_IDLE);
			#endif
		}
		laserTask->start(laserTask_SchedParams, connections.laserTask.cpuAffinity);
	} else {
		laserTask->start();
	}
}

/**
 * Start all timers contained in this component
 */
void SmartLaserLMS200Server::startAllTimers() {
}


Smart::TaskTriggerSubject* SmartLaserLMS200Server::getInputTaskTriggerFromString(const std::string &client)
{
	if(client == "BaseStateIn") return baseStateInInputTaskTrigger;
	
	return NULL;
}


void SmartLaserLMS200Server::init(int argc, char *argv[])
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
		
		SmartLaserLMS200ServerPortFactoryInterface *acePortFactory = portFactoryRegistry["ACE_SmartSoft"];
		if(acePortFactory == 0) {
			std::cerr << "ERROR: acePortFactory NOT instantiated -> exit(-1)" << std::endl;
			exit(-1);
		}
		
		// this pointer is used for backwards compatibility (deprecated: should be removed as soon as all patterns, including coordination, are moved to port-factory)
		SmartACE::SmartComponent *component = dynamic_cast<SmartLaserLMS200ServerAcePortFactory*>(acePortFactory)->getComponentImpl();
		
		std::cout << "ComponentDefinition SmartLaserLMS200Server is named " << connections.component.name << std::endl;
		
		if(connections.component.useLogger == true) {
			//FIXME: use logging
			//Smart::LOGGER->openLogFileInFolder("data/"+connections.component.name);
			//Smart::LOGGER->startLogging();
		}

		// create event-test handlers (if needed)
		
		// create server ports
		// TODO: set minCycleTime from Ini-file
		laserQueryServiceAnsw = portFactoryRegistry[connections.laserQueryServiceAnsw.roboticMiddleware]->createLaserQueryServiceAnsw(connections.laserQueryServiceAnsw.serviceName);
		laserQueryServiceAnswInputTaskTrigger = new Smart::QueryServerTaskTrigger<CommBasicObjects::CommVoid, CommBasicObjects::CommMobileLaserScan>(laserQueryServiceAnsw);
		laserServiceOut = portFactoryRegistry[connections.laserServiceOut.roboticMiddleware]->createLaserServiceOut(connections.laserServiceOut.serviceName);
		laserServiceOutWrapper = new LaserServiceOutWrapper(laserServiceOut);
		
		// create client ports
		baseStateIn = portFactoryRegistry[connections.baseStateIn.roboticMiddleware]->createBaseStateIn();
		
		// create InputTaskTriggers and UpcallManagers
		baseStateInInputCollector = new BaseStateInInputCollector(baseStateIn);
		baseStateInInputTaskTrigger = new Smart::InputTaskTrigger<CommBasicObjects::CommBaseState>(baseStateInInputCollector);
		baseStateInUpcallManager = new BaseStateInUpcallManager(baseStateInInputCollector);
		
		// create input-handler
		
		// create request-handlers
		laserQueryServiceAnswHandler = new LaserQueryServiceAnswHandler(laserQueryServiceAnsw);
		
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
		if(connections.baseStateIn.roboticMiddleware == "ACE_SmartSoft") {
			//FIXME: this must also work with other implementations
			dynamic_cast<SmartACE::PushClient<CommBasicObjects::CommBaseState>*>(baseStateIn)->add(wiringSlave, connections.baseStateIn.wiringName);
		}
		
		// create parameter slave
		param = new SmartACE::ParameterSlave(component, &paramHandler);
		
		
		// create Task LaserTask
		laserTask = new LaserTask(component);
		// configure input-links
		baseStateInUpcallManager->attach(laserTask);
		// configure task-trigger (if task is configurable)
		
		
		// link observers with subjects
		laserTask->attach_interaction_observer(laserQueryServiceAnswHandler);
	} catch (const std::exception &ex) {
		std::cerr << "Uncaught std exception" << ex.what() << std::endl;
	} catch (...) {
		std::cerr << "Uncaught exception" << std::endl;
	}
}

// run the component
void SmartLaserLMS200Server::run()
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
void SmartLaserLMS200Server::fini()
{
	// unlink all observers
	laserTask->detach_interaction_observer(laserQueryServiceAnswHandler);
	
	// destroy all task instances
	// unlink all UpcallManagers
	baseStateInUpcallManager->detach(laserTask);
	// unlink the TaskTrigger
	if(laserTaskTrigger != NULL){
		laserTaskTrigger->detach(laserTask);
		delete laserTask;
	}

	// destroy all input-handler

	// destroy InputTaskTriggers and UpcallManagers
	delete baseStateInInputTaskTrigger;
	delete baseStateInUpcallManager;
	delete baseStateInInputCollector;

	// destroy client ports
	delete baseStateIn;

	// destroy request-handlers
	delete laserQueryServiceAnswHandler;

	// destroy server ports
	delete laserQueryServiceAnswInputTaskTrigger;
	delete laserQueryServiceAnsw;
	delete laserServiceOutWrapper;
	delete laserServiceOut;
	
	// destroy event-test handlers (if needed)
	
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

void SmartLaserLMS200Server::loadParameter(int argc, char *argv[])
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
		} else if(parameter.searchFile("SmartLaserLMS200Server.ini", parameterfile)) {
			parameterFileFound = true;
			std::cout << "load SmartLaserLMS200Server.ini parameter file\n";
			parameter.addFile(parameterfile);
		} else {
			std::cout << "WARNING: SmartLaserLMS200Server.ini parameter file not found! (using default values or command line arguments)\n";
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
		
		// load parameters for client BaseStateIn
		parameter.getBoolean("BaseStateIn", "initialConnect", connections.baseStateIn.initialConnect);
		parameter.getString("BaseStateIn", "serviceName", connections.baseStateIn.serviceName);
		parameter.getString("BaseStateIn", "serverName", connections.baseStateIn.serverName);
		parameter.getString("BaseStateIn", "wiringName", connections.baseStateIn.wiringName);
		parameter.getInteger("BaseStateIn", "interval", connections.baseStateIn.interval);
		if(parameter.checkIfParameterExists("BaseStateIn", "roboticMiddleware")) {
			parameter.getString("BaseStateIn", "roboticMiddleware", connections.baseStateIn.roboticMiddleware);
		}
		
		// load parameters for server LaserQueryServiceAnsw
		parameter.getString("LaserQueryServiceAnsw", "serviceName", connections.laserQueryServiceAnsw.serviceName);
		if(parameter.checkIfParameterExists("LaserQueryServiceAnsw", "roboticMiddleware")) {
			parameter.getString("LaserQueryServiceAnsw", "roboticMiddleware", connections.laserQueryServiceAnsw.roboticMiddleware);
		}
		// load parameters for server LaserServiceOut
		parameter.getString("LaserServiceOut", "serviceName", connections.laserServiceOut.serviceName);
		if(parameter.checkIfParameterExists("LaserServiceOut", "roboticMiddleware")) {
			parameter.getString("LaserServiceOut", "roboticMiddleware", connections.laserServiceOut.roboticMiddleware);
		}
		
		// load parameters for task LaserTask
		if(parameter.checkIfParameterExists("LaserTask", "scheduler")) {
			parameter.getString("LaserTask", "scheduler", connections.laserTask.scheduler);
		}
		if(parameter.checkIfParameterExists("LaserTask", "priority")) {
			parameter.getInteger("LaserTask", "priority", connections.laserTask.priority);
		}
		if(parameter.checkIfParameterExists("LaserTask", "cpuAffinity")) {
			parameter.getInteger("LaserTask", "cpuAffinity", connections.laserTask.cpuAffinity);
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
